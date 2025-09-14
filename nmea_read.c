// nmea_read.c
// Reads NMEA from a serial port, waits up to timeout, parses time/date/lat/lon,
// prints a single-line JSON object to stderr and also prints date/time separately.
//
// Usage: ./nmea_read [-v] [dev] [baud] [timeout_sec] [TZ]
// Examples:
//   ./nmea_read
//   ./nmea_read -v /dev/ttyS0 115200 5 America/Chicago
//   ./nmea_read /dev/ttyS0 115200 15 CST6CDT
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include <sys/select.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <math.h>

#ifndef B115200
#error "Your system headers do not define B115200"
#endif

typedef struct {
    int has_time;
    int has_date;
    int has_lat;
    int has_lon;
    int hour, minute, second; // UTC time from NMEA
    int year, month, day;     // UTC date from RMC/ZDA (YYYY,1-12,1-31)
    double lat, lon;          // decimal degrees
} gps_info_t;

/* prototypes */
static speed_t baud_to_speed_t(unsigned long baud);
static void usage(const char *prog);
static int setup_serial(const char* dev, unsigned long baud);
static int checksum_ok(const char* line);
static int parse_hhmmss(const char* s, int* H, int* M, int* S);
static int parse_ddmmyy(const char* s, int* Y, int* M, int* D);
static int starts_with_talker(const char* line, const char* type3);
static int is_valid_fix_char(char c);
static double parse_degmin_to_decimal(const char* fld, char hemi);
static void parse_rmc(char* line, gps_info_t* info);
static void parse_gga(char* line, gps_info_t* info);
static void parse_gll(char* line, gps_info_t* info);
static void parse_zda(char* line, gps_info_t* info);
static void parse_line(char* line_in, gps_info_t* info);
static void timespec_add_sec(struct timespec* t, int sec);
static int timespec_cmp(const struct timespec* a, const struct timespec* b);
static void timespec_sub(const struct timespec* a, const struct timespec* b, struct timespec* out);

/* implementation */
static speed_t baud_to_speed_t(unsigned long baud) {
    switch (baud) {
        case 4800: return B4800;
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
#ifdef B230400
        case 230400: return B230400;
#endif
#ifdef B460800
        case 460800: return B460800;
#endif
        default: return B115200;
    }
}
static void usage(const char *prog) {
    fprintf(stderr,
        "Usage: %s [-v] [dev] [baud] [timeout_sec] [TZ]\n"
        "  -v          - verbose: echo raw NMEA lines to stderr\n"
        "  dev         - serial device (default /dev/ttyS0)\n"
        "  baud        - baud rate (default 115200)\n"
        "  timeout_sec - overall timeout in seconds (default 5)\n"
        "  TZ          - optional timezone (IANA like America/Chicago or POSIX like EST5EDT)\n"
        "Examples:\n"
        "  %s\n"
        "  %s -v /dev/ttyS0 115200 5 America/Chicago\n",
        prog, prog, prog);
}
static int setup_serial(const char* dev, unsigned long baud) {
    int fd = open(dev, O_RDONLY | O_NOCTTY);
    if (fd < 0) {
        fprintf(stderr, "Failed to open %s: %s\n", dev, strerror(errno));
        return -1;
    }
    struct termios tio;
    memset(&tio, 0, sizeof(tio));
    if (tcgetattr(fd, &tio) != 0) {
        fprintf(stderr, "tcgetattr failed: %s\n", strerror(errno));
        close(fd);
        return -1;
    }
    cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
#ifdef CRTSCTS
    tio.c_cflag &= ~CRTSCTS;
#endif
    tio.c_iflag &= ~(IXON | IXOFF | IXANY);
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0;
    speed_t spd = baud_to_speed_t(baud);
    cfsetispeed(&tio, spd);
    cfsetospeed(&tio, spd);
    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
        fprintf(stderr, "tcsetattr failed: %s\n", strerror(errno));
        close(fd);
        return -1;
    }
    tcflush(fd, TCIFLUSH);
    return fd;
}
/* Validate checksum only if '*' exists. If no '*' present we fall through and allow parsing. */
static int checksum_ok(const char* line) {
    const char* star = strrchr(line, '*');
    if (!star) return 1; /* no checksum present: allow parsing */
    if ((size_t)(strlen(star+1)) < 2) return 0;
    unsigned int given = 0;
    if (sscanf(star + 1, "%2x", &given) != 1) return 0;
    unsigned char sum = 0;
    for (const char* p = line + 1; p < star; ++p) sum ^= (unsigned char)(*p);
    return sum == (unsigned char)given;
}
static int parse_hhmmss(const char* s, int* H, int* M, int* S) {
    if (!s || !*s) return 0;
    int h=0,m=0;
    double sec=0.0;
    if (sscanf(s, "%2d%2d%lf", &h, &m, &sec) < 2) return 0;
    if (h < 0 || h > 23 || m < 0 || m > 59) return 0;
    int isec = (int)floor(sec + 0.5);
    if (isec < 0) isec = 0;
    if (isec > 59) isec = 59;
    *H = h; *M = m; *S = isec;
    return 1;
}
static int parse_ddmmyy(const char* s, int* Y, int* M, int* D) {
    if (!s || strlen(s) < 6) return 0;
    int d=0,m=0,y=0;
    if (sscanf(s, "%2d%2d%2d", &d, &m, &y) != 3) return 0;
    if (d < 1 || d > 31 || m < 1 || m > 12) return 0;
    int year = (y >= 80) ? (1900 + y) : (2000 + y);
    *Y = year; *M = m; *D = d;
    return 1;
}
static int starts_with_talker(const char* line, const char* type3) {
    if (!line || line[0] != '$') return 0;
    if (strlen(line) < 6) return 0;
    return strncmp(line + 3, type3, 3) == 0;
}
static int is_valid_fix_char(char c) { return c == 'A'; }
static double parse_degmin_to_decimal(const char* fld, char hemi) {
    if (!fld || !*fld) return NAN;
    double v = atof(fld);
    if (v == 0.0 && (fld[0] != '0')) return NAN;
    double deg = floor(v / 100.0);
    double min = v - deg * 100.0;
    double dec = deg + (min / 60.0);
    if (hemi == 'S' || hemi == 'W') dec = -dec;
    return dec;
}
/* $--RMC,time,status,lat,NS,lon,EW,sog,cog,date,... */
static void parse_rmc(char* line, gps_info_t* info) {
    char* star = strchr(line, '*'); if (star) *star = '\0';
    char* save = NULL;
    char* tok = strtok_r(line, ",", &save);
    int idx = 0;
    const char* fields[24] = {0};
    while (tok && idx < 24) { fields[idx++] = tok; tok = strtok_r(NULL, ",", &save); }
    const char* times = (idx > 1) ? fields[1] : NULL;
    const char* status = (idx > 2) ? fields[2] : NULL;
    const char* lats = (idx > 3) ? fields[3] : NULL;
    const char* ns   = (idx > 4) ? fields[4] : NULL;
    const char* lons = (idx > 5) ? fields[5] : NULL;
    const char* ew   = (idx > 6) ? fields[6] : NULL;
    const char* dates= (idx > 9) ? fields[9] : NULL;
    if (!info->has_time && times && *times) {
        int H,M,S;
        if (parse_hhmmss(times, &H, &M, &S)) {
            info->hour = H; info->minute = M; info->second = S; info->has_time = 1;
        }
    }
    if (!info->has_date && dates && *dates) {
        int Y,M,D;
        if (parse_ddmmyy(dates, &Y, &M, &D)) {
            info->year = Y; info->month = M; info->day = D; info->has_date = 1;
        }
    }
    int fix_ok = (status && is_valid_fix_char(status[0]));
    if (fix_ok) {
        if (!info->has_lat && lats && ns && *lats && *ns) {
            double lat = parse_degmin_to_decimal(lats, ns[0]);
            if (!isnan(lat)) { info->lat = lat; info->has_lat = 1; }
        }
        if (!info->has_lon && lons && ew && *lons && *ew) {
            double lon = parse_degmin_to_decimal(lons, ew[0]);
            if (!isnan(lon)) { info->lon = lon; info->has_lon = 1; }
        }
    }
}
/* $--GGA,time,lat,NS,lon,EW,fixQ,... */
static void parse_gga(char* line, gps_info_t* info) {
    char* star = strchr(line, '*'); if (star) *star = '\0';
    char* save = NULL;
    char* tok = strtok_r(line, ",", &save);
    int idx = 0;
    const char* fields[20] = {0};
    while (tok && idx < 20) { fields[idx++] = tok; tok = strtok_r(NULL, ",", &save); }
    const char* times = (idx > 1) ? fields[1] : NULL;
    const char* lats  = (idx > 2) ? fields[2] : NULL;
    const char* ns    = (idx > 3) ? fields[3] : NULL;
    const char* lons  = (idx > 4) ? fields[4] : NULL;
    const char* ew    = (idx > 5) ? fields[5] : NULL;
    const char* fixq  = (idx > 6) ? fields[6] : NULL;
    if (!info->has_time && times && *times) {
        int H,M,S;
        if (parse_hhmmss(times, &H, &M, &S)) {
            info->hour = H; info->minute = M; info->second = S; info->has_time = 1;
        }
    }
    int fix_ok = (fixq && *fixq != '0');
    if (fix_ok) {
        if (!info->has_lat && lats && ns && *lats && *ns) {
            double lat = parse_degmin_to_decimal(lats, ns[0]);
            if (!isnan(lat)) { info->lat = lat; info->has_lat = 1; }
        }
        if (!info->has_lon && lons && ew && *lons && *ew) {
            double lon = parse_degmin_to_decimal(lons, ew[0]);
            if (!isnan(lon)) { info->lon = lon; info->has_lon = 1; }
        }
    }
}
/* $--GLL,lat,NS,lon,EW,time,status*hh */
static void parse_gll(char* line, gps_info_t* info) {
    char* star = strchr(line, '*'); if (star) *star = '\0';
    char* save = NULL;
    char* tok = strtok_r(line, ",", &save);
    int idx = 0;
    const char* fields[16] = {0};
    while (tok && idx < 16) { fields[idx++] = tok; tok = strtok_r(NULL, ",", &save); }
    const char* lats  = (idx > 1) ? fields[1] : NULL;
    const char* ns    = (idx > 2) ? fields[2] : NULL;
    const char* lons  = (idx > 3) ? fields[3] : NULL;
    const char* ew    = (idx > 4) ? fields[4] : NULL;
    const char* times = (idx > 5) ? fields[5] : NULL;
    const char* status= (idx > 6) ? fields[6] : NULL;
    if (!info->has_time && times && *times) {
        int H,M,S;
        if (parse_hhmmss(times, &H, &M, &S)) {
            info->hour = H; info->minute = M; info->second = S; info->has_time = 1;
        }
    }
    int fix_ok = (status && is_valid_fix_char(status[0]));
    if (fix_ok) {
        if (!info->has_lat && lats && ns && *lats && *ns) {
            double lat = parse_degmin_to_decimal(lats, ns[0]);
            if (!isnan(lat)) { info->lat = lat; info->has_lat = 1; }
        }
        if (!info->has_lon && lons && ew && *lons && *ew) {
            double lon = parse_degmin_to_decimal(lons, ew[0]);
            if (!isnan(lon)) { info->lon = lon; info->has_lon = 1; }
        }
    }
}
/* $--ZDA,hhmmss,dd,mm,yyyy,local_zone_hours,local_zone_minutes*hh */
static void parse_zda(char* line, gps_info_t* info) {
    char* star = strchr(line, '*'); if (star) *star = '\0';
    char* save = NULL;
    char* tok = strtok_r(line, ",", &save);
    int idx = 0;
    const char* fields[12] = {0};
    while (tok && idx < 12) { fields[idx++] = tok; tok = strtok_r(NULL, ",", &save); }
    const char* times = (idx > 1) ? fields[1] : NULL;
    const char* day   = (idx > 2) ? fields[2] : NULL;
    const char* mon   = (idx > 3) ? fields[3] : NULL;
    const char* year  = (idx > 4) ? fields[4] : NULL;
    if (!info->has_time && times && *times) {
        int H,M,S;
        if (parse_hhmmss(times, &H, &M, &S)) {
            info->hour = H; info->minute = M; info->second = S; info->has_time = 1;
        }
    }
    if (!info->has_date && day && mon && year && *day && *mon && *year) {
        int D = atoi(day);
        int M = atoi(mon);
        int Y = atoi(year);
        if (Y >= 1900 && M >= 1 && M <= 12 && D >= 1 && D <= 31) {
            info->year = Y; info->month = M; info->day = D; info->has_date = 1;
        }
    }
}
/* parse a single NMEA line (CR/LF removed) */
static void parse_line(char* line_in, gps_info_t* info) {
    size_t len = strlen(line_in);
    while (len > 0 && (line_in[len-1] == '\r' || line_in[len-1] == '\n')) {
        line_in[--len] = '\0';
    }
    if (len < 1 || line_in[0] != '$') return;
    /* If '*' exists, validate checksum; if no '*', allow parsing */
    if (!checksum_ok(line_in)) return;
    /* copy because tokenizers mutate */
    char buf[256];
    strncpy(buf, line_in, sizeof(buf)-1);
    buf[sizeof(buf)-1] = '\0';
    if (starts_with_talker(buf, "RMC")) parse_rmc(buf, info);
    else if (starts_with_talker(buf, "GGA")) parse_gga(buf, info);
    else if (starts_with_talker(buf, "GLL")) parse_gll(buf, info);
    else if (starts_with_talker(buf, "ZDA")) parse_zda(buf, info);
}
/* timespec helpers */
static void timespec_add_sec(struct timespec* t, int sec) { t->tv_sec += sec; }
static int timespec_cmp(const struct timespec* a, const struct timespec* b) {
    if (a->tv_sec != b->tv_sec) return (a->tv_sec < b->tv_sec) ? -1 : 1;
    if (a->tv_nsec != b->tv_nsec) return (a->tv_nsec < b->tv_nsec) ? -1 : 1;
    return 0;
}
static void timespec_sub(const struct timespec* a, const struct timespec* b, struct timespec* out) {
    out->tv_sec = a->tv_sec - b->tv_sec;
    out->tv_nsec = a->tv_nsec - b->tv_nsec;
    if (out->tv_nsec < 0) {
        out->tv_nsec += 1000000000L;
        out->tv_sec -= 1;
    }
}

int main(int argc, char** argv) {
    const char* dev = "/dev/ttyS0";
    unsigned long baud = 115200;
    int timeout_sec = 5;
    const char* tz_arg = NULL;
    int verbosity = 0;

    /* parse options and positionals */
    int pos_idx = 0;
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            usage(argv[0]);
            return 0;
        } else if (strcmp(argv[i], "-v") == 0) {
            verbosity++;
        } else {
            /* positional */
            if (pos_idx == 0) dev = argv[i];
            else if (pos_idx == 1) baud = strtoul(argv[i], NULL, 10);
            else if (pos_idx == 2) timeout_sec = atoi(argv[i]);
            else if (pos_idx == 3) tz_arg = argv[i];
            pos_idx++;
        }
    }

    if (timeout_sec <= 0) timeout_sec = 5;
    int fd = setup_serial(dev, baud);
    if (fd < 0) return 1;

    gps_info_t info = {0};
    char linebuf[1024];
    size_t linelen = 0;

    struct timespec start, deadline;
    clock_gettime(CLOCK_MONOTONIC, &start);
    deadline = start;
    timespec_add_sec(&deadline, timeout_sec);

    /* Wait for time, lat, lon, AND date (so utc_date is filled), bounded by timeout */
    while (!(info.has_time && info.has_lat && info.has_lon && info.has_date)) {
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        if (timespec_cmp(&now, &deadline) >= 0) break;

        struct timespec left;
        timespec_sub(&deadline, &now, &left);
        struct timeval tv;
        tv.tv_sec = left.tv_sec;
        tv.tv_usec = left.tv_nsec / 1000;

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        int sel = select(fd + 1, &rfds, NULL, NULL, &tv);
        if (sel < 0) {
            if (errno == EINTR) continue;
            fprintf(stderr, "select error: %s\n", strerror(errno));
            break;
        } else if (sel == 0) {
            continue;
        }

        if (FD_ISSET(fd, &rfds)) {
            char buf[256];
            ssize_t n = read(fd, buf, sizeof(buf));
            if (n < 0) {
                if (errno == EAGAIN || errno == EINTR) continue;
                fprintf(stderr, "read error: %s\n", strerror(errno));
                break;
            } else if (n == 0) {
                break;
            } else {
                for (ssize_t i = 0; i < n; ++i) {
                    char c = buf[i];
                    if (c == '\n') {
                        linebuf[linelen] = '\0';
                        if (linelen > 0) {
                            if (verbosity > 0) fprintf(stderr, "RAW: %s\n", linebuf);
                            parse_line(linebuf, &info);
                        }
                        linelen = 0;
                    } else if (c != '\r') {
                        if (linelen + 1 < sizeof(linebuf)) linebuf[linelen++] = c;
                        else linelen = 0;
                    }
                }
            }
        }
    }

    /* Prepare UTC date/time strings (or null) */
    char utc_date[32] = {0};
    char utc_time[16] = {0};
    if (info.has_date) snprintf(utc_date, sizeof(utc_date), "%04d-%02d-%02d", info.year, info.month, info.day);
    if (info.has_time) snprintf(utc_time, sizeof(utc_time), "%02d:%02d:%02d", info.hour, info.minute, info.second);

    /* Prepare local date/time strings if requested and time available */
    char local_date[32] = {0};
    char local_time[32] = {0};
    char local_dt[128] = {0}; // combined with TZ
    int have_local = 0;
    if (tz_arg && info.has_time) {
        struct tm tm_utc = {0};
        if (info.has_date) {
            tm_utc.tm_year = info.year - 1900;
            tm_utc.tm_mon  = info.month - 1;
            tm_utc.tm_mday = info.day;
        } else {
            /* fall back to today's UTC date if no date was parsed */
            time_t now = time(NULL);
            struct tm tm_now;
            gmtime_r(&now, &tm_now);
            tm_utc.tm_year = tm_now.tm_year;
            tm_utc.tm_mon  = tm_now.tm_mon;
            tm_utc.tm_mday = tm_now.tm_mday;
        }
        tm_utc.tm_hour = info.hour;
        tm_utc.tm_min  = info.minute;
        tm_utc.tm_sec  = info.second;

        time_t t_utc = timegm(&tm_utc);

        char oldtz_buf[512];
        const char* oldtz = getenv("TZ");
        int had_oldtz = 0;
        if (oldtz) {
            strncpy(oldtz_buf, oldtz, sizeof(oldtz_buf)-1);
            oldtz_buf[sizeof(oldtz_buf)-1] = '\0';
            had_oldtz = 1;
        }

        setenv("TZ", tz_arg, 1);
        tzset();

        struct tm tm_local;
        localtime_r(&t_utc, &tm_local);
        if (strftime(local_date, sizeof(local_date), "%Y-%m-%d", &tm_local) != 0 &&
            strftime(local_time, sizeof(local_time), "%H:%M:%S", &tm_local) != 0 &&
            strftime(local_dt, sizeof(local_dt), "%Y-%m-%d %H:%M:%S %Z", &tm_local) != 0) {
            have_local = 1;
        } else {
            local_date[0] = '\0';
            local_time[0] = '\0';
            local_dt[0] = '\0';
            have_local = 0;
        }

        if (had_oldtz) setenv("TZ", oldtz_buf, 1);
        else unsetenv("TZ");
        tzset();
    }

    /* Emit JSON to stderr */
    fprintf(stderr, "{");
    if (info.has_date) fprintf(stderr, "\"utc_date\":\"%s\",", utc_date); else fprintf(stderr, "\"utc_date\":null,");
    if (info.has_time) fprintf(stderr, "\"utc_time\":\"%s\",", utc_time); else fprintf(stderr, "\"utc_time\":null,");
    if (have_local) fprintf(stderr, "\"local_date\":\"%s\",", local_date); else fprintf(stderr, "\"local_date\":null,");
    if (have_local) fprintf(stderr, "\"local_time\":\"%s\",", local_time); else fprintf(stderr, "\"local_time\":null,");
    if (have_local) fprintf(stderr, "\"local_datetime\":\"%s\",", local_dt); else fprintf(stderr, "\"local_datetime\":null,");
    if (info.has_lat) fprintf(stderr, "\"lat\":%.6f,", info.lat); else fprintf(stderr, "\"lat\":null,");
    if (info.has_lon) fprintf(stderr, "\"lon\":%.6f", info.lon); else fprintf(stderr, "\"lon\":null");
    fprintf(stderr, "}\n");

    /* Also print human-friendly lines */
    if (info.has_date) fprintf(stderr, "date=%s\n", utc_date); else fprintf(stderr, "date=unavailable\n");
    if (info.has_time) fprintf(stderr, "time=%sZ\n", utc_time); else fprintf(stderr, "time=unavailable\n");
    if (have_local) {
        fprintf(stderr, "local_date=%s\n", local_date);
        fprintf(stderr, "local_time=%s\n", local_time);
        fprintf(stderr, "local_datetime=%s\n", local_dt);
    }

    close(fd);
    return 0;
}
