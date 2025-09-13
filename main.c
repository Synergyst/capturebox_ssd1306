#include <linux/i2c-dev.h>
#include <stdint.h>
#include <stdlib.h>
#include <inttypes.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <time.h>
#include "ssd1306.h"
#include "linux_i2c.h"
#include "image.h"

// Persistent resolution file used by the library
static const char* RES_FILE = "/tmp/.ssd1306_oled_type";

// Ctrl-C flag
static volatile sig_atomic_t g_stop = 0;

// Forward decls
static void print_help(void);
static int read_saved_resolution(int* out_cols, int* out_lines);

// SIGINT/SIGTERM handler
static void on_sigint(int sig) {
  (void)sig;
  g_stop = 1;
}

static void print_help() {
  printf("ssd1306_bin usage:\n\n");
  printf("-I <WxH>   initialize OLED resolution (128x32 | 128x64 | 64x48)\n");
  printf("-n <bus>   I2C device node number (default 1) e.g. /dev/i2c-<bus>\n");
  printf("-d <0|1>   display off/on\n");
  printf("-i <0|1>   invert off/on\n");
  printf("-r <0|180> rotate orientation (0 or 180)\n");
  printf("-c[<row>]  clear all if no arg; else clear specific row (page)\n");
  printf("-x <col>   set cursor X (column)\n");
  printf("-y <page>  set cursor Y (page, 8px tall)\n");
  printf("-f <0|1>   font: 0=5x7 small, 1=8x8 normal\n");
  printf("-l <text>  write a single line at current cursor\n");
  printf("-m <text>  write multi-line string (use \\n for line breaks)\n");
  printf("-A         run animation demo (Ctrl-C to stop)\n");
  printf("-S <ms>    animation frame delay (default 30 ms)\n");
  printf("-T         show built-in test image (128x64 1bpp)\n");
  printf("-P         run INA219 power meter UI (exclusive with -A/-T/-H)\n");
  printf("-B <bus>   INA219 I2C bus number (default 10)\n");
  printf("-G <addr>  INA219 I2C address (hex 0x43 or decimal, default 0x43)\n");
  printf("-U <ms>    INA219 sample/update period in ms (default 2000 ms)\n");
  printf("-E <list>  enable fields: comma list of psu,load,curr,power,pct,vid,fmt or 'none' (default all)\n");
  printf("-b         show battery icon on bottom row (independent of -E)\n");
  printf("-C <name>  charge icon style: plug|plus|check|bolt_h (default plug)\n");
  printf("-F         flash between fill-only and icon overlay (half period each)\n");
  printf("-z         invert charge sense (swap charging/discharging detection)\n");
  printf("-k <px>    battery icon X offset in pixels (default centered)\n");
  printf("-w <px>    battery icon width (14..24, default 18)\n");
  printf("-H         run video signal monitor (exclusive with -A/-T/-P)\n");
  printf("-V <list>  video devices to check (comma list), default /dev/video0\n");
  printf("-Q <ms>    video poll interval (default 2000 ms)\n");
  printf("-j <px>    video icon X offset in pixels (default 0)\n");
  printf("-h         this help\n");
}

// ---------------- INA219 (Power Meter) ----------------
#define INA219_REG_CONFIG          0x00
#define INA219_REG_SHUNT_VOLTAGE   0x01
#define INA219_REG_BUS_VOLTAGE     0x02
#define INA219_REG_POWER           0x03
#define INA219_REG_CURRENT         0x04
#define INA219_REG_CALIBRATION     0x05

// Config fields (matching your Python settings)
#define INA219_RANGE_16V           0x00
#define INA219_GAIN_80MV           0x01
#define INA219_ADC_12BIT_32S       0x0D
#define INA219_MODE_SANDB_CONT     0x07

typedef struct {
  int      fd;
  uint8_t  addr;
  double   current_lsb;  // mA per bit (matches your Python)
  double   power_lsb;    // W per bit
  uint16_t cal_value;
} ina219_t;

static int ina219_open(ina219_t* dev, int bus, uint8_t addr) {
  if (!dev) return -1;
  memset(dev, 0, sizeof(*dev));
  dev->addr = addr;
  char filename[32];
  snprintf(filename, sizeof(filename), "/dev/i2c-%d", bus);
  dev->fd = open(filename, O_RDWR);
  if (dev->fd < 0) return -1;
  if (ioctl(dev->fd, I2C_SLAVE, addr) < 0) {
    close(dev->fd);
    dev->fd = -1;
    return -1;
  }
  return 0;
}

static void ina219_close(ina219_t* dev) {
  if (dev && dev->fd >= 0) {
    close(dev->fd);
    dev->fd = -1;
  }
}

static int ina219_write16(ina219_t* dev, uint8_t reg, uint16_t val) {
  uint8_t buf[3];
  buf[0] = reg;
  buf[1] = (uint8_t)((val >> 8) & 0xFF);  // MSB first
  buf[2] = (uint8_t)(val & 0xFF);
  ssize_t w = write(dev->fd, buf, 3);
  return (w == 3) ? 0 : -1;
}

static int ina219_read16(ina219_t* dev, uint8_t reg, uint16_t* out) {
  if (!out) return -1;
  uint8_t r = reg;
  if (write(dev->fd, &r, 1) != 1) return -1;
  uint8_t buf[2];
  if (read(dev->fd, buf, 2) != 2) return -1;
  *out = (uint16_t)((buf[0] << 8) | buf[1]);  // MSB first
  return 0;
}

// Configure like your Python set_calibration_16V_5A()
static int ina219_calibrate_16V_5A(ina219_t* dev) {
  if (!dev) return -1;
  dev->cal_value   = 26868;      // matches Python
  dev->current_lsb = 0.1524;     // mA per bit
  dev->power_lsb   = 0.003048;   // W per bit
  if (ina219_write16(dev, INA219_REG_CALIBRATION, dev->cal_value) != 0)
    return -1;
  uint16_t config = 0;
  config |= (uint16_t)(INA219_RANGE_16V & 0x01) << 13;
  config |= (uint16_t)(INA219_GAIN_80MV & 0x03) << 11;
  config |= (uint16_t)(INA219_ADC_12BIT_32S & 0x0F) << 7;  // bus adc
  config |= (uint16_t)(INA219_ADC_12BIT_32S & 0x0F) << 3;  // shunt adc
  config |= (uint16_t)(INA219_MODE_SANDB_CONT & 0x07);
  if (ina219_write16(dev, INA219_REG_CONFIG, config) != 0)
    return -1;
  return 0;
}

static double ina219_get_shunt_voltage_V(ina219_t* dev) {
  uint16_t raw = 0;
  ina219_write16(dev, INA219_REG_CALIBRATION, dev->cal_value);
  if (ina219_read16(dev, INA219_REG_SHUNT_VOLTAGE, &raw) != 0) return 0.0;
  int16_t sraw = (raw > 32767) ? (int16_t)(raw - 65535) : (int16_t)raw;
  return (double)sraw * 1e-5;  // volts
}

static double ina219_get_bus_voltage_V(ina219_t* dev) {
  ina219_write16(dev, INA219_REG_CALIBRATION, dev->cal_value);
  uint16_t raw = 0;
  if (ina219_read16(dev, INA219_REG_BUS_VOLTAGE, &raw) != 0) return 0.0;
  raw >>= 3;
  return (double)raw * 0.004;  // volts
}

static double ina219_get_current_mA(ina219_t* dev) {
  uint16_t raw = 0;
  if (ina219_read16(dev, INA219_REG_CURRENT, &raw) != 0) return 0.0;
  int16_t sraw = (raw > 32767) ? (int16_t)(raw - 65535) : (int16_t)raw;
  return (double)sraw * dev->current_lsb;  // mA
}

static double ina219_get_power_W(ina219_t* dev) {
  ina219_write16(dev, INA219_REG_CALIBRATION, dev->cal_value);
  uint16_t raw = 0;
  if (ina219_read16(dev, INA219_REG_POWER, &raw) != 0) return 0.0;
  int16_t sraw = (raw > 32767) ? (int16_t)(raw - 65535) : (int16_t)raw;
  return (double)sraw * dev->power_lsb;  // W
}

// ---------------- SSD1306 helpers (page-level blit) ----------------
static uint8_t ssd1306_blit_full_pages(const uint8_t* pages, int w, int h) {
  if (!pages || w <= 0 || h <= 0 || (h % 8) != 0) return 1;
  int page_count = h / 8;
  uint8_t tx[1 + 128];
  tx[0] = SSD1306_DATA_CONTROL_BYTE;
  ssd1306_oled_set_mem_mode(SSD1306_PAGE_MODE);
  for (int p = 0; p < page_count; ++p) {
    if (ssd1306_oled_set_XY(0, (uint8_t)p) != 0) return 1;
    memcpy(&tx[1], &pages[p * w], (size_t)w);
    if (_i2c_write(tx, (int16_t)(1 + w)) != 0) return 1;
  }
  return 0;
}

static uint8_t ssd1306_blit_from_01(const uint8_t* img01, int w, int h) {
  if (!img01 || w <= 0 || h <= 0 || (h % 8) != 0 || w > 128) return 1;
  static uint8_t pagebuf[128 * 64 / 8];
  for (int p = 0; p < h / 8; ++p) {
    for (int x = 0; x < w; ++x) {
      uint8_t b = 0;
      for (int bit = 0; bit < 8; ++bit) {
        int y = p * 8 + bit;
        if (img01[y * w + x]) b |= (uint8_t)(1u << bit);
      }
      pagebuf[p * w + x] = b;
    }
  }
  return ssd1306_blit_full_pages(pagebuf, w, h);
}

static uint8_t oled_write_page_row(uint8_t x, uint8_t page, const uint8_t* bytes, uint8_t w) {
  if (!bytes || w == 0) return 1;
  if (w > 128) return 1;
  if (ssd1306_oled_set_XY(x, page) != 0) return 1;
  uint8_t tx[1 + 128];
  tx[0] = SSD1306_DATA_CONTROL_BYTE;
  memcpy(&tx[1], bytes, w);
  return _i2c_write(tx, (int16_t)(1 + w));
}

// ---------------- Animation demo ----------------
static void build_row_vertical_bar(uint8_t* row, int cols, int bar_x) {
  memset(row, 0x00, (size_t)cols);
  if (bar_x >= 0 && bar_x < cols) row[bar_x] = 0xFF;
}

static void build_row_horizontal_line(uint8_t* row, int cols, int page, int y) {
  memset(row, 0x00, (size_t)cols);
  int target_page = y >> 3;
  if (page == target_page) {
    uint8_t mask = (uint8_t)(1u << (y & 7));
    memset(row, mask, (size_t)cols);
  }
}

static void build_row_diagonal_pixel(uint8_t* row, int cols, int page, int x, int y) {
  memset(row, 0x00, (size_t)cols);
  int target_page = y >> 3;
  if (page == target_page && x >= 0 && x < cols) {
    uint8_t mask = (uint8_t)(1u << (y & 7));
    row[x] = mask;
  }
}

static void run_animation_loop(int cols, int lines, unsigned frame_delay_ms) {
  if (cols <= 0 || lines <= 0) return;
  ssd1306_oled_set_mem_mode(SSD1306_PAGE_MODE);
  ssd1306_oled_clear_screen();
  const int pages = lines / 8;
  uint8_t row_buf[128];
  if (cols > (int)sizeof(row_buf)) cols = (int)sizeof(row_buf);
  uint32_t t = 0;
  while (!g_stop) {
    int span = (cols > lines ? cols : lines);
    int mode = (t / (uint32_t)span) % 3;
    int bar_x = (int)(t % (uint32_t)cols);
    int y     = (int)(t % (uint32_t)lines);
    int x     = (int)(t % (uint32_t)cols);
    for (int p = 0; p < pages; ++p) {
      switch (mode) {
        case 0:
          build_row_vertical_bar(row_buf, cols, bar_x);
          break;
        case 1:
          build_row_horizontal_line(row_buf, cols, p, y);
          break;
        case 2:
        default:
          build_row_diagonal_pixel(row_buf, cols, p, x, y);
          break;
      }
      if (oled_write_page_row(0, (uint8_t)p, row_buf, (uint8_t)cols) != 0) return;
    }
    t++;
    if (frame_delay_ms == 0) frame_delay_ms = 1;
    usleep(frame_delay_ms * 1000);
  }
}

// ---------------- Power Meter UI with differential updates ----------------
#define METER_FONT_SIZE SSD1306_FONT_SMALL

static inline int meter_char_width(void) {
  return (METER_FONT_SIZE == SSD1306_FONT_SMALL) ? 6 : 8;
}

static void meter_draw_run(uint8_t row_page, int x_chars, const char* s, int run_len) {
  if (run_len <= 0) return;
  int cw = meter_char_width();
  int x_px = x_chars * cw;
  ssd1306_oled_set_XY((uint8_t)x_px, row_page);
  char buf[64];
  while (run_len > 0) {
    int n = (run_len < (int)sizeof(buf) - 1) ? run_len : (int)sizeof(buf) - 1;
    memcpy(buf, s, (size_t)n);
    buf[n] = '\0';
    ssd1306_oled_write_line(METER_FONT_SIZE, buf);
    s += n;
    x_chars += n;
    x_px = x_chars * cw;
    if (run_len > n) ssd1306_oled_set_XY((uint8_t)x_px, row_page);
    run_len -= n;
  }
}

static void meter_diff_and_draw(uint8_t row_page, int x_chars, char* prev, const char* now) {
  int old_len = (int)strlen(prev);
  int new_len = (int)strlen(now);
  int max_len = (old_len > new_len) ? old_len : new_len;
  int i = 0;
  while (i < max_len) {
    char new_ch = (i < new_len) ? now[i] : ' ';
    char old_ch = (i < old_len) ? prev[i] : ' ';
    if (new_ch == old_ch) {
      i++;
      continue;
    }
    int run_start = i;
    int run_len = 1;
    i++;
    while (i < max_len) {
      char new_ch_i = (i < new_len) ? now[i] : ' ';
      char old_ch_i = (i < old_len) ? prev[i] : ' ';
      if (new_ch_i != old_ch_i) {
        run_len++;
        i++;
      } else {
        break;
      }
    }
    int seg1_len = 0;
    if (run_start < new_len) {
      seg1_len = new_len - run_start;
      if (seg1_len > run_len) seg1_len = run_len;
      meter_draw_run(row_page, x_chars + run_start, &now[run_start], seg1_len);
    }
    int seg2_len = run_len - seg1_len;
    if (seg2_len > 0) {
      int offset = 0;
      char spaces[64];
      memset(spaces, ' ', sizeof(spaces));
      while (seg2_len > 0) {
        int chunk = (seg2_len > (int)sizeof(spaces)) ? (int)sizeof(spaces) : seg2_len;
        meter_draw_run(row_page, x_chars + run_start + seg1_len + offset, spaces, chunk);
        seg2_len -= chunk;
        offset += chunk;
      }
    }
  }
  strncpy(prev, now, (size_t)max_len);
  prev[new_len] = '\0';
}

// ---------------- Dynamic fields and battery + video icons ----------------
// Field mask bits
#define FLD_PSU    (1u << 0)
#define FLD_LOAD   (1u << 1)
#define FLD_CURR   (1u << 2)
#define FLD_POWER  (1u << 3)
#define FLD_PCT    (1u << 4)
#define FLD_NONE   (1u << 5)
#define FLD_VID    (1u << 6)
#define FLD_FMT    (1u << 7)

static inline uint8_t default_field_mask(void) {
  return (FLD_PSU | FLD_LOAD | FLD_CURR | FLD_POWER | FLD_PCT | FLD_VID | FLD_FMT);
}

static uint8_t parse_enabled_fields(const char* s) {
  if (!s || !*s) return default_field_mask();
  uint8_t mask = 0;
  int saw_none = 0;
  char buf[128];
  strncpy(buf, s, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';
  char* tok = strtok(buf, ",");
  while (tok) {
    while (*tok == ' ' || *tok == '\t') tok++;
    if (!strcasecmp(tok, "none")) { saw_none = 1; mask = FLD_NONE; break; }
    if (!strcasecmp(tok, "psu"))   mask |= FLD_PSU;
    else if (!strcasecmp(tok, "load"))  mask |= FLD_LOAD;
    else if (!strcasecmp(tok, "curr"))  mask |= FLD_CURR;
    else if (!strcasecmp(tok, "power")) mask |= FLD_POWER;
    else if (!strcasecmp(tok, "pct"))   mask |= FLD_PCT;
    else if (!strcasecmp(tok, "vid"))   mask |= FLD_VID;
    else if (!strcasecmp(tok, "fmt"))   mask |= FLD_FMT;
    tok = strtok(NULL, ",");
  }
  if (saw_none) return FLD_NONE;
  return (mask == 0) ? default_field_mask() : mask;
}

// video list type for multi-device support
#define MAX_VDEVS 8
typedef struct {
  int count;
  char devs[MAX_VDEVS][64];
} video_list_t;

static void trim_leading(char** s) {
  while (**s == ' ' || **s == '\t') (*s)++;
}

static void build_video_list(const char* list, video_list_t* out) {
  if (!out) return;
  memset(out, 0, sizeof(*out));
  if (!list || !*list) {
    snprintf(out->devs[0], sizeof(out->devs[0]), "/dev/video0");
    out->count = 1;
    return;
  }
  char buf[256];
  strncpy(buf, list, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';
  char* tok = strtok(buf, ",");
  while (tok && out->count < MAX_VDEVS) {
    trim_leading(&tok);
    if (strncmp(tok, "/dev/video", 10) != 0) {
      snprintf(out->devs[out->count], sizeof(out->devs[out->count]), "/dev/video%s", tok);
    } else {
      snprintf(out->devs[out->count], sizeof(out->devs[out->count]), "%s", tok);
    }
    out->count++;
    tok = strtok(NULL, ",");
  }
  if (out->count == 0) {
    snprintf(out->devs[0], sizeof(out->devs[0]), "/dev/video0");
    out->count = 1;
  }
}

static void meter_draw_static_labels_dyn_for_vlist(const video_list_t* vlist, uint8_t enabled_mask, uint8_t max_rows) {
  if (!(enabled_mask & FLD_NONE)) {
    const char* lab_psu   = "SUPP(V):";
    const char* lab_load  = "LOAD(V):";
    const char* lab_curr  = "CUR(mA):";
    const char* lab_power = "POWR(W):";
    const char* lab_pct   = "BATT(%):";
    uint8_t row = 0;
    if ((enabled_mask & FLD_PSU)   && row < max_rows) { ssd1306_oled_set_XY(0, row); ssd1306_oled_write_line(METER_FONT_SIZE, (char*)lab_psu);   row++; }
    if ((enabled_mask & FLD_LOAD)  && row < max_rows) { ssd1306_oled_set_XY(0, row); ssd1306_oled_write_line(METER_FONT_SIZE, (char*)lab_load);  row++; }
    if ((enabled_mask & FLD_CURR)  && row < max_rows) { ssd1306_oled_set_XY(0, row); ssd1306_oled_write_line(METER_FONT_SIZE, (char*)lab_curr);  row++; }
    if ((enabled_mask & FLD_POWER) && row < max_rows) { ssd1306_oled_set_XY(0, row); ssd1306_oled_write_line(METER_FONT_SIZE, (char*)lab_power); row++; }

    // VID / FMT handling: if vlist has multiple devices, create VID0:, FMT0:, VID1:, FMT1: etc.
    if (vlist && vlist->count > 1) {
      for (int i = 0; i < vlist->count && row < max_rows; ++i) {
        char lab[16];
        snprintf(lab, sizeof(lab), "VID%d:", i);
        ssd1306_oled_set_XY(0, row);
        ssd1306_oled_write_line(METER_FONT_SIZE, lab);
        row++;
        if (row >= max_rows) break;
        snprintf(lab, sizeof(lab), "FMT%d:", i);
        ssd1306_oled_set_XY(0, row);
        ssd1306_oled_write_line(METER_FONT_SIZE, lab);
        row++;
      }
    } else {
      // Single device or none: use legacy labels
      if ((enabled_mask & FLD_VID) && row < max_rows) { ssd1306_oled_set_XY(0, row); ssd1306_oled_write_line(METER_FONT_SIZE, (char*)"VID:"); row++; }
      if ((enabled_mask & FLD_FMT) && row < max_rows) { ssd1306_oled_set_XY(0, row); ssd1306_oled_write_line(METER_FONT_SIZE, (char*)"FMT:"); row++; }
    }

    if ((enabled_mask & FLD_PCT)   && row < max_rows) { ssd1306_oled_set_XY(0, row); ssd1306_oled_write_line(METER_FONT_SIZE, (char*)lab_pct);   row++; }
  }
}

typedef struct {
  char psu[24];
  char load[24];
  char curr[24];
  char power[24];
  char pct[24];
  // keep single vid/fmt for single-device mode only
  char vid[24];
  char fmt[24];
  int  initialized;
} meter_prev_t;

// updated to support multi-device VID/FMT expansion
static uint8_t meter_update_values_dyn(meter_prev_t* st,
                                       const video_list_t* vlist,
                                       uint8_t enabled_mask,
                                       uint8_t max_rows,
                                       int start_chars,
                                       double busV, double shuntV,
                                       double current_mA, double power_W,
                                       double percent,
                                       const char vid_texts[][24],
                                       const char fmt_texts[][24],
                                       int vid_count) {
  if (!st) return 0;
  char now_psu[24], now_load[24], now_curr[24], now_power[24], now_pct[24];
  char now_vid[24], now_fmt[24];
  snprintf(now_psu,   sizeof(now_psu),   "%1.3f", busV + shuntV);
  snprintf(now_load,  sizeof(now_load),  "%1.3f", busV);
  snprintf(now_curr,  sizeof(now_curr),  "%d", (int)current_mA);
  snprintf(now_power, sizeof(now_power), "%2.3f", power_W);
  if (percent >= 90.0) snprintf(now_pct, sizeof(now_pct), "%3.1f", percent);
  else                 snprintf(now_pct, sizeof(now_pct), "%3.2f", percent);
  snprintf(now_vid,   sizeof(now_vid),   "%s", (vid_count > 0 && vid_texts) ? vid_texts[0] : "");
  snprintf(now_fmt,   sizeof(now_fmt),   "%s", (vid_count > 0 && fmt_texts) ? fmt_texts[0] : "");

  if (!st->initialized) {
    meter_draw_static_labels_dyn_for_vlist(vlist, enabled_mask, max_rows);
    st->psu[0] = st->load[0] = st->curr[0] = st->power[0] = st->pct[0] = st->vid[0] = st->fmt[0] = '\0';
    st->initialized = 1;
  }

  // Compute label lengths so VID/FMT values can be placed immediately after their labels
  const int vid_label_chars_single = (int)strlen("VID:"); // 4
  const int fmt_label_chars_single = (int)strlen("FMT:"); // 4

  uint8_t row = 0;
  if ((enabled_mask & FLD_PSU)   && row < max_rows) { meter_diff_and_draw(row, start_chars, st->psu,   now_psu);   row++; }
  if ((enabled_mask & FLD_LOAD)  && row < max_rows) { meter_diff_and_draw(row, start_chars, st->load,  now_load);  row++; }
  if ((enabled_mask & FLD_CURR)  && row < max_rows) { meter_diff_and_draw(row, start_chars, st->curr,  now_curr);  row++; }
  if ((enabled_mask & FLD_POWER) && row < max_rows) { meter_diff_and_draw(row, start_chars, st->power, now_power); row++; }

  // For VID/FMT:
  if (vlist && vlist->count > 1) {
    // Multi-device: expand VIDn/FMTn pairs into rows
    for (int i = 0; i < vid_count && row < max_rows; ++i) {
      // VIDi: -> write value immediately after "VIDn:" label, which is 5 chars (e.g., "VID0:")
      char val[24];
      snprintf(val, sizeof(val), "%s", vid_texts[i]);
      int label_chars = (int)strlen("VID0:"); // 5
      // set position and write (clear first to avoid remnants)
      //ssd1306_oled_clear_line(row);
      ssd1306_oled_set_XY((uint8_t)(label_chars * meter_char_width()), row);
      ssd1306_oled_write_line(METER_FONT_SIZE, val);
      row++;
      if (row >= max_rows) break;
      // FMTi:
      //ssd1306_oled_clear_line(row);
      ssd1306_oled_set_XY((uint8_t)(label_chars * meter_char_width()), row);
      ssd1306_oled_write_line(METER_FONT_SIZE, fmt_texts[i]);
      row++;
    }
  } else {
    // Single device: behave like original code for VID/FMT
    if ((enabled_mask & FLD_VID)   && row < max_rows) { meter_diff_and_draw(row, vid_label_chars_single, st->vid,   now_vid);   row++; }
    if ((enabled_mask & FLD_FMT)   && row < max_rows) { meter_diff_and_draw(row, fmt_label_chars_single, st->fmt,   now_fmt);   row++; }
  }

  if ((enabled_mask & FLD_PCT)   && row < max_rows) { meter_diff_and_draw(row, start_chars, st->pct,   now_pct);   row++; }
  return row;
}

// Charge icon styles
typedef enum {
  CHARGE_ICON_PLUG = 0,
  CHARGE_ICON_PLUS = 1,
  CHARGE_ICON_CHECK = 2,
  CHARGE_ICON_BOLT_H = 3
} ChargeIconStyle;

// Pixel carving helpers
static inline void carve_bit(uint8_t* cols, int width, int x, int y) {
  if (x < 0 || x >= width) return;
  if (y < 0 || y > 7) return;
  cols[x] &= (uint8_t)~(1u << y);
}

static void carve_hline(uint8_t* cols, int width, int x0, int x1, int y, int inner_left, int inner_right, int thick) {
  if (y < 0 || y > 7) return;
  if (x0 > x1) { int t = x0; x0 = x1; x1 = t; }
  if (x0 < inner_left) x0 = inner_left;
  if (x1 > inner_right) x1 = inner_right;
  for (int x = x0; x <= x1; ++x) {
    for (int dy = -thick + 1; dy <= thick - 1; ++dy) {
      int yy = y + dy;
      if (yy > 0 && yy < 7) carve_bit(cols, width, x, yy);
    }
  }
}

static void carve_vline(uint8_t* cols, int width, int x, int y0, int y1, int inner_left, int inner_right, int thick) {
  if (x < inner_left || x > inner_right) return;
  if (y0 > y1) { int t = y0; y0 = y1; y1 = t; }
  if (y0 < 1) y0 = 1;
  if (y1 > 6) y1 = 6;
  for (int y = y0; y <= y1; ++y) {
    for (int dx = -thick + 1; dx <= thick - 1; ++dx) {
      int xx = x + dx;
      if (xx >= inner_left && xx <= inner_right) carve_bit(cols, width, xx, y);
    }
  }
}

static inline void clamp_center(int* cx, int* cy, int inner_left, int inner_right, int top, int bottom) {
  if (*cx < inner_left + 2) *cx = inner_left + 2;
  if (*cx > inner_right - 2) *cx = inner_right - 2;
  if (*cy < top + 2) *cy = top + 2;
  if (*cy > bottom - 2) *cy = bottom - 2;
}

// Overlays
static void overlay_plug(uint8_t* cols, int width, int inner_left, int inner_right, int top, int bottom, int cx, int cy) {
  clamp_center(&cx, &cy, inner_left, inner_right, top, bottom);
  int thick = 1;
  for (int x = cx - 3; x <= cx; ++x) {
    carve_vline(cols, width, x, cy - 2, cy + 3, inner_left, inner_right, 1);
  }
  carve_vline(cols, width, cx + 3, cy - 1, cy - 1, inner_left, inner_right, 2);
  carve_vline(cols, width, cx + 3, cy + 2, cy + 2, inner_left, inner_right, 2);
  carve_hline(cols, width, cx - 1, cx, cy, inner_left, inner_right, thick);
}

static void overlay_plus(uint8_t* cols, int width, int inner_left, int inner_right, int top, int bottom, int cx, int cy) {
  clamp_center(&cx, &cy, inner_left, inner_right, top, bottom);
  int thick = 1;
  carve_hline(cols, width, cx - 2, cx + 2, cy, inner_left, inner_right, thick);
  carve_vline(cols, width, cx, cy - 2, cy + 2, inner_left, inner_right, thick);
}

static void overlay_check(uint8_t* cols, int width, int inner_left, int inner_right, int top, int bottom, int cx, int cy) {
  clamp_center(&cx, &cy, inner_left, inner_right, top, bottom);
  int thick = 1;
  carve_bit(cols, width, cx - 3, cy + 1);
  carve_bit(cols, width, cx - 2, cy);
  carve_bit(cols, width, cx - 1, cy);
  carve_bit(cols, width, cx,     cy - 1);
  carve_bit(cols, width, cx + 1, cy - 2);
  carve_bit(cols, width, cx + 2, cy - 3);
  carve_hline(cols, width, cx - 3, cx - 1, cy, inner_left, inner_right, thick);
  carve_hline(cols, width, cx, cx + 2, cy - 2, inner_left, inner_right, thick);
}

static void overlay_bolt_h(uint8_t* cols, int width, int inner_left, int inner_right, int top, int bottom, int cx, int cy) {
  clamp_center(&cx, &cy, inner_left, inner_right, top, bottom);
  int thick = 1;
  carve_bit(cols, width, cx - 3, cy - 2);
  carve_bit(cols, width, cx - 2, cy - 1);
  carve_bit(cols, width, cx - 1, cy);
  carve_bit(cols, width, cx,     cy);
  carve_bit(cols, width, cx + 1, cy + 1);
  carve_bit(cols, width, cx + 2, cy + 2);
  carve_hline(cols, width, cx - 3, cx - 1, cy - 1, inner_left, inner_right, thick);
  carve_hline(cols, width, cx, cx + 2, cy + 1, inner_left, inner_right, thick);
}

// Compute fill width without rounding up; only 100% gives a full bar
static inline int calc_fill_w(int inner_w, int level) {
  if (level <= 0) return 0;
  if (level >= 100) return inner_w;
  return (inner_w * level) / 100;
}

// Battery icon with style overlay
static void draw_battery_icon_with_style(uint8_t last_page,
                                         int x_px,
                                         int width_px,
                                         int level,
                                         int charging,
                                         ChargeIconStyle style) {
  if (width_px < 14) width_px = 14;
  if (width_px > 24) width_px = 24;
  if (level < 0) level = 0;
  if (level > 100) level = 100;
  uint8_t cols[24];
  memset(cols, 0x00, (size_t)width_px);
  const int left = 0, right = width_px - 1, top = 0, bottom = 7;
  const int tip_w = 2, tip_x0 = right - (tip_w - 1), tip_y0 = 2, tip_y1 = 5;
  const int body_left = left, body_right = tip_x0 - 1;
  const int inner_left = body_left + 1, inner_right = body_right - 1;
  for (int x = body_left; x <= body_right; ++x) {
    cols[x] |= (1u << top);
    cols[x] |= (1u << bottom);
  }
  for (int y = top; y <= bottom; ++y) {
    cols[body_left]  |= (1u << y);
    cols[body_right] |= (1u << y);
  }
  for (int x = tip_x0; x <= right; ++x) {
    for (int y = tip_y0; y <= tip_y1; ++y) cols[x] |= (1u << y);
  }
  const int inner_w = (inner_right >= inner_left) ? (inner_right - inner_left + 1) : 0;
  const int fill_w = calc_fill_w(inner_w, level);
  for (int x = inner_left; x < inner_left + fill_w; ++x) {
    for (int y = top + 1; y <= bottom - 1; ++y) cols[x] |= (1u << y);
  }
  if (charging) {
    int cx = inner_left + inner_w / 2;
    int cy = (top + bottom) / 2;
    switch (style) {
      case CHARGE_ICON_PLUG:   overlay_plug(cols, width_px, inner_left, inner_right, top, bottom, cx, cy);   break;
      case CHARGE_ICON_PLUS:   overlay_plus(cols, width_px, inner_left, inner_right, top, bottom, cx, cy);   break;
      case CHARGE_ICON_CHECK:  overlay_check(cols, width_px, inner_left, inner_right, top, bottom, cx, cy);  break;
      case CHARGE_ICON_BOLT_H:
      default:                 overlay_bolt_h(cols, width_px, inner_left, inner_right, top, bottom, cx, cy); break;
    }
  }
  oled_write_page_row((uint8_t)x_px, last_page, cols, (uint8_t)width_px);
}

// Legacy battery (fill only or with bolt cut-out)
static void draw_battery_icon(uint8_t last_page, int x_px, int width_px, int level, int charging) {
  if (width_px < 14) width_px = 14;
  if (width_px > 24) width_px = 24;
  if (level < 0) level = 0;
  if (level > 100) level = 100;
  uint8_t cols[24];
  memset(cols, 0x00, (size_t)width_px);
  const int left = 0, right = width_px - 1, top = 0, bottom = 7;
  const int tip_w = 2, tip_x0 = right - (tip_w - 1), tip_y0 = 2, tip_y1 = 5;
  const int body_left = left, body_right = tip_x0 - 1;
  const int inner_left = body_left + 1, inner_right = body_right - 1;
  for (int x = body_left; x <= body_right; ++x) {
    cols[x] |= (1u << top);
    cols[x] |= (1u << bottom);
  }
  for (int y = top; y <= bottom; ++y) {
    cols[body_left]  |= (1u << y);
    cols[body_right] |= (1u << y);
  }
  for (int x = tip_x0; x <= right; ++x) {
    for (int y = tip_y0; y <= tip_y1; ++y) cols[x] |= (1u << y);
  }
  const int inner_w = (inner_right >= inner_left) ? (inner_right - inner_left + 1) : 0;
  const int fill_w = calc_fill_w(inner_w, level);
  for (int x = inner_left; x < inner_left + fill_w; ++x) {
    for (int y = top + 1; y <= bottom - 1; ++y) cols[x] |= (1u << y);
  }
  if (charging) {
    int cx = inner_left + inner_w / 2;
    if (cx < inner_left + 2) cx = inner_left + 2;
    if (cx > inner_right - 2) cx = inner_right - 2;
    const int y_coords[6] = { top+1, top+2, top+3, top+4, top+5, top+6 };
    const int x_coords[6] = { cx-2, cx-1, cx, cx, cx+1, cx+2 };
    for (int i = 0; i < 6; ++i) {
      int by = y_coords[i];
      if (by <= top || by >= bottom) continue;
      int bx = x_coords[i];
      if (bx < inner_left || bx > inner_right) continue;
      cols[bx] &= (uint8_t)~(1u << by);
    }
  }
  oled_write_page_row((uint8_t)x_px, last_page, cols, (uint8_t)width_px);
}

typedef struct {
  int prev_level;
  int prev_chg;
  int prev_style;
  int drawn;
} batt_state_style_t;

static void battery_icon_update_style(batt_state_style_t* st,
                                      uint8_t last_page,
                                      int x_px,
                                      int width_px,
                                      int level,
                                      int charging,
                                      ChargeIconStyle style) {
  if (!st) return;
  if (!st->drawn || level != st->prev_level || charging != st->prev_chg || style != st->prev_style) {
    draw_battery_icon_with_style(last_page, x_px, width_px, level, charging, style);
    st->prev_level = level;
    st->prev_chg = charging;
    st->prev_style = style;
    st->drawn = 1;
  }
}

typedef struct {
  int prev_level;
  int prev_chg;
  int drawn;
} batt_state_t;

static void battery_icon_update(batt_state_t* st, uint8_t last_page, int x_px, int width_px, int level, int charging) {
  if (!st) return;
  if (!st->drawn || level != st->prev_level || charging != st->prev_chg) {
    draw_battery_icon(last_page, x_px, width_px, level, charging);
    st->prev_level = level;
    st->prev_chg = charging;
    st->drawn = 1;
  }
}

// ---------------- Video signal parsing and icons ----------------
typedef enum {
  VID_UNKNOWN = 0,
  VID_ABSENT  = 1,
  VID_PRESENT = 2
} video_state_t;

static video_state_t parse_video_status_for_dev(const char* devnode, char* fmt_out, size_t fmt_len) {
  if (fmt_out && fmt_len) fmt_out[0] = '\0';
  if (!devnode) return VID_UNKNOWN;
  char cmd[256];
  snprintf(cmd, sizeof(cmd), "v4l2-ctl --log-status -d %s 2>&1", devnode);
  FILE* fp = popen(cmd, "r");
  if (!fp) return VID_UNKNOWN;
  char line[512];
  video_state_t st = VID_UNKNOWN;
  while (fgets(line, sizeof(line), fp)) {
    if (strstr(line, "No video detected")) {
      st = VID_ABSENT;
    }
    if (strstr(line, "Detected format:")) {
      st = VID_PRESENT;
      char* p = strstr(line, "Detected format:");
      p += strlen("Detected format:");
      while (*p == ' ' || *p == '\t') p++;
      size_t n = strcspn(p, " (");
      if (fmt_out && fmt_len) {
        if (n >= fmt_len) n = fmt_len - 1;
        memcpy(fmt_out, p, n);
        fmt_out[n] = '\0';
      }
    }
  }
  pclose(fp);
  return st;
}

static video_state_t parse_video_status_multi(const char* list, char* fmt_out, size_t fmt_len) {
  if (fmt_out && fmt_len) fmt_out[0] = '\0';
  if (!list || !*list) return parse_video_status_for_dev("/dev/video0", fmt_out, fmt_len);
  char buf[256];
  strncpy(buf, list, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';
  video_state_t agg = VID_ABSENT;
  char* tok = strtok(buf, ",");
  while (tok) {
    while (*tok == ' ' || *tok == '\t') tok++;
    char dev[64];
    strncpy(dev, tok, sizeof(dev) - 1);
    dev[sizeof(dev) - 1] = '\0';
    // If user passed just a number, prefix /dev/video
    if (strncmp(dev, "/dev/video", 10) != 0) {
      char tmp[64];
      snprintf(tmp, sizeof(tmp), "/dev/video%s", dev);
      strncpy(dev, tmp, sizeof(dev) - 1);
      dev[sizeof(dev) - 1] = '\0';
    }
    char fmt_local[128];
    video_state_t st = parse_video_status_for_dev(dev, fmt_local, sizeof(fmt_local));
    if (st == VID_PRESENT) {
      agg = VID_PRESENT;
      if (fmt_out && fmt_len && fmt_out[0] == '\0') {
        snprintf(fmt_out, fmt_len, "%s", fmt_local);
      }
      // keep scanning to update 'agg' but we already have one present
    } else if (st == VID_UNKNOWN && agg == VID_ABSENT) {
      agg = VID_UNKNOWN;
    }
    tok = strtok(NULL, ",");
  }
  return agg;
}

// Small boxed signal icons on the bottom row
#define VICON_W  12
#define VICON_SP 2

static void draw_video_icon_box(uint8_t last_page, int x_px, int is_ok) {
  uint8_t cols[VICON_W];
  memset(cols, 0x00, sizeof(cols));
  // border
  for (int x = 0; x < VICON_W; ++x) {
    cols[x] |= (1u << 0);
    cols[x] |= (1u << 7);
  }
  for (int y = 0; y < 8; ++y) {
    cols[0]           |= (1u << y);
    cols[VICON_W - 1] |= (1u << y);
  }
  if (is_ok) {
    // a small checkmark inside
    int cx = 4;
    cols[cx - 1] |= (1u << 4);
    cols[cx]     |= (1u << 5);
    cols[cx + 1] |= (1u << 5);
    cols[cx + 2] |= (1u << 4);
    cols[cx + 3] |= (1u << 3);
    cols[cx + 4] |= (1u << 2);
  } else {
    // an 'X' inside
    for (int i = 2; i <= 5; ++i) {
      //int x1 = 2 + (i - 2);
      int x1 = 4 + (i - 2);
      //int x2 = (VICON_W - 3) - (i - 2);
      int x2 = (VICON_W - 3) - i;
      cols[x1] |= (1u << i);
      cols[x2] |= (1u << i);
    }
  }
  oled_write_page_row((uint8_t)x_px, last_page, cols, (uint8_t)sizeof(cols));
}

typedef struct {
  int prev_state;
  int drawn;
} vid_box_state_t;

static void video_box_update(vid_box_state_t* st, uint8_t last_page, int x_px, video_state_t s) {
  if (!st) return;
  if (!st->drawn || st->prev_state != (int)s) {
    draw_video_icon_box(last_page, x_px, (s == VID_PRESENT));
    st->prev_state = (int)s;
    st->drawn = 1;
  }
}

// ---------------- Power meter loop with video overlays ----------------
static void run_power_meter_loop(int bus,
                                 uint8_t addr,
                                 unsigned period_ms,
                                 int oled_cols,
                                 int oled_lines,
                                 uint8_t fields_mask,
                                 int show_batt_icon,
                                 int invert_charge,
                                 int batt_x_offset,
                                 int batt_width_px,
                                 int flash_icon,
                                 ChargeIconStyle icon_style,
                                 const char* video_devs,
                                 unsigned video_poll_ms,
                                 int vid_icon_x_offset) {
  ina219_t meter;
  if (ina219_open(&meter, bus, addr) != 0) {
    fprintf(stderr, "INA219 open failed (bus %d addr 0x%02X)\n", bus, addr);
    return;
  }
  if (ina219_calibrate_16V_5A(&meter) != 0) {
    fprintf(stderr, "INA219 calibration failed\n");
    ina219_close(&meter);
    return;
  }
  ssd1306_oled_set_mem_mode(SSD1306_PAGE_MODE);
  meter_prev_t prev = {0};
  batt_state_t bstate_fill = {0};
  batt_state_style_t bstate_icon = {0};

  uint8_t page_count = (uint8_t)(oled_lines / 8);
  if (page_count == 0) page_count = 1;
  uint8_t last_page = (uint8_t)(page_count - 1);
  uint8_t max_text_rows = show_batt_icon ? last_page : page_count;
  const int start_chars = 8;

  int icon_x = batt_x_offset;
  if (icon_x < 0) {
    icon_x = (oled_cols - batt_width_px) / 2;
    if (icon_x < 0) icon_x = 0;
  }

  // Build video device list and initialize per-device icon states
  video_list_t vlist;
  build_video_list(video_devs, &vlist);
  vid_box_state_t vicons[MAX_VDEVS];
  memset(vicons, 0, sizeof(vicons));

  // storage for per-device text values (OK/NO, fmt)
  char vid_texts[MAX_VDEVS][24];
  char fmt_texts[MAX_VDEVS][24];
  for (int i = 0; i < MAX_VDEVS; ++i) { vid_texts[i][0] = '\0'; fmt_texts[i][0] = '\0'; }

  // default video poll timer
  unsigned last_video_poll_ms = 0;

  while (!g_stop) {
    // Timing base
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    unsigned now_ms = (unsigned)(ts.tv_sec * 1000u + ts.tv_nsec / 1000000u);

    // Poll video status at requested interval
    int do_video_poll = 0;
    if (last_video_poll_ms == 0 || (now_ms - last_video_poll_ms) >= (video_poll_ms ? video_poll_ms : 2000)) {
      do_video_poll = 1;
      last_video_poll_ms = now_ms;
    }

    static char agg_fmt_buf[64];
    static video_state_t agg_state = VID_UNKNOWN;

    if (do_video_poll) {
      agg_state = VID_ABSENT;
      agg_fmt_buf[0] = '\0';
      for (int i = 0; i < vlist.count; ++i) {
        char fmt_local[128];
        video_state_t st = parse_video_status_for_dev(vlist.devs[i], fmt_local, sizeof(fmt_local));
        // draw/update each device’s box
        int x_box = vid_icon_x_offset + i * (VICON_W + VICON_SP);
        if (x_box < oled_cols) {
          video_box_update(&vicons[i], last_page, x_box, st);
        }
        // prepare per-device text values
        if (st == VID_PRESENT) {
          snprintf(vid_texts[i], sizeof(vid_texts[i]), "OK");
          snprintf(fmt_texts[i], sizeof(fmt_texts[i]), "%s", fmt_local);
          if (agg_state != VID_PRESENT) {
            agg_state = VID_PRESENT;
            snprintf(agg_fmt_buf, sizeof(agg_fmt_buf), "%s", fmt_local);
          }
        } else if (st == VID_ABSENT) {
          snprintf(vid_texts[i], sizeof(vid_texts[i]), "NO");
          fmt_texts[i][0] = '\0';
        } else {
          snprintf(vid_texts[i], sizeof(vid_texts[i]), "UNK");
          fmt_texts[i][0] = '\0';
          if (agg_state == VID_ABSENT) agg_state = VID_UNKNOWN;
        }
      }
    }

    // Read INA219 once per period
    double busV    = ina219_get_bus_voltage_V(&meter);
    double shuntV  = ina219_get_shunt_voltage_V(&meter);
    double curr_mA = ina219_get_current_mA(&meter);
    double power_W = ina219_get_power_W(&meter);
    double percent = (busV - 3.0) / 1.2 * 100.0;
    if (percent > 100.0) percent = 100.0;
    if (percent < 0.0)   percent = 0.0;

    // Draw labels (first time) and diff-update values
    if (!(fields_mask & FLD_NONE)) {
      meter_update_values_dyn(&prev, &vlist, fields_mask, max_text_rows, start_chars,
                              busV, shuntV, curr_mA, power_W, percent,
                              (const char(*)[24])vid_texts, (const char(*)[24])fmt_texts, vlist.count);
    }

    // Battery icon update
    if (show_batt_icon) {
      int charging = ((curr_mA < 0.0) ? 1 : 0) ^ (invert_charge ? 1 : 0);
      if (flash_icon) {
        unsigned half_ms = (period_ms / 2u);
        if (half_ms < 50u) half_ms = 50u;
        draw_battery_icon(last_page, icon_x, batt_width_px, (int)(percent + 0.5), 0);
        usleep(half_ms * 1000);
        draw_battery_icon_with_style(last_page, icon_x, batt_width_px, (int)(percent + 0.5), charging, icon_style);
        usleep(half_ms * 1000);
      } else {
        if (charging) {
          battery_icon_update_style(&bstate_icon, last_page, icon_x, batt_width_px, (int)(percent + 0.5), 1, icon_style);
        } else {
          battery_icon_update(&bstate_fill, last_page, icon_x, batt_width_px, (int)(percent + 0.5), 0);
        }
        if (period_ms < 50u) period_ms = 50u;
        usleep(period_ms * 1000);
      }
    } else {
      if (period_ms < 50u) period_ms = 50u;
      usleep(period_ms * 1000);
    }

    // Console logging (optional)
    printf("PSU Voltage:   %6.3f V\n", busV + shuntV);
    printf("Shunt Voltage: %9.6f V\n", shuntV);
    printf("Load Voltage:  %6.3f V\n", busV);
    printf("Current:       %6.3f A\n", curr_mA / 1000.0);
    printf("Power:         %6.3f W\n", power_W);
    printf("Percent:       %3.1f%%\n", percent);
    printf("Video State:   %s\n", (agg_state == VID_PRESENT) ? "PRESENT" : ((agg_state == VID_ABSENT) ? "ABSENT" : "UNKNOWN"));
    if (agg_state == VID_PRESENT) printf("Detected fmt:  %s\n", agg_fmt_buf);
    printf("\n");
  }
  ina219_close(&meter);
}

// ---------------- Video-only monitor loop ----------------
// Shows per-device labels VID0:/FMT0:, VID1:/FMT1: and draws boxed icons on bottom row
static void run_video_monitor_loop(int oled_cols,
                                   int oled_lines,
                                   const char* video_devs,
                                   unsigned video_poll_ms,
                                   uint8_t fields_mask,
                                   int vid_icon_x_offset) {
  ssd1306_oled_set_mem_mode(SSD1306_PAGE_MODE);
  uint8_t page_count = (uint8_t)(oled_lines / 8);
  if (page_count == 0) page_count = 1;
  uint8_t last_page = (uint8_t)(page_count - 1);

  video_list_t vlist;
  build_video_list(video_devs, &vlist);

  vid_box_state_t vicons[MAX_VDEVS];
  memset(vicons, 0, sizeof(vicons));

  // Text rows available (reserve bottom page for icons)
  uint8_t max_text_rows = (last_page > 0) ? last_page : 0;
  const int start_chars = 6;

  // draw static labels once: VID0:, FMT0:, VID1:, FMT1:, ...
  uint8_t label_row = 0;
  for (int i = 0; i < vlist.count && label_row < max_text_rows; ++i) {
    char lab[16];
    snprintf(lab, sizeof(lab), "VID%d:", i);
    ssd1306_oled_set_XY(0, label_row);
    ssd1306_oled_write_line(METER_FONT_SIZE, lab);
    label_row++;
    if (label_row >= max_text_rows) break;
    snprintf(lab, sizeof(lab), "FMT%d:", i);
    ssd1306_oled_set_XY(0, label_row);
    ssd1306_oled_write_line(METER_FONT_SIZE, lab);
    label_row++;
  }

  // per-device values
  char vid_texts[MAX_VDEVS][24];
  char fmt_texts[MAX_VDEVS][24];
  for (int i = 0; i < MAX_VDEVS; ++i) { vid_texts[i][0] = '\0'; fmt_texts[i][0] = '\0'; }

  while (!g_stop) {
    for (int i = 0; i < vlist.count; ++i) {
      char fmt_local[96];
      video_state_t st = parse_video_status_for_dev(vlist.devs[i], fmt_local, sizeof(fmt_local));

      // Icons on bottom page, horizontally spaced
      int x_box = vid_icon_x_offset + i * (VICON_W + VICON_SP);
      if (x_box < oled_cols) {
        video_box_update(&vicons[i], last_page, x_box, st);
      }

      // per-device text
      if (!(fields_mask & FLD_NONE)) {
        const char* vid_text = (st == VID_PRESENT) ? "OK" : ((st == VID_ABSENT) ? "NO" : "UNK");
        snprintf(vid_texts[i], sizeof(vid_texts[i]), "%s", vid_text);
        if (st == VID_PRESENT) snprintf(fmt_texts[i], sizeof(fmt_texts[i]), "%s", fmt_local);
        else fmt_texts[i][0] = '\0';
      }
    }

    // write per-device values immediately after labels
    int row = 0;
    for (int i = 0; i < vlist.count && row < max_text_rows; ++i) {
      // VIDn value
      int label_chars = (int)strlen("VID0:"); // 5
      ssd1306_oled_clear_line(row);
      ssd1306_oled_set_XY((uint8_t)(label_chars * meter_char_width()), (uint8_t)row);
      ssd1306_oled_write_line(METER_FONT_SIZE, vid_texts[i]);
      row++;
      if (row >= max_text_rows) break;
      // FMTn value
      ssd1306_oled_clear_line(row);
      ssd1306_oled_set_XY((uint8_t)(label_chars * meter_char_width()), (uint8_t)row);
      ssd1306_oled_write_line(METER_FONT_SIZE, fmt_texts[i]);
      row++;
    }

    if (video_poll_ms < 100u) video_poll_ms = 100u;
    usleep(video_poll_ms * 1000);
  }
}

// ---------------- Resolution helper ----------------
static int read_saved_resolution(int* out_cols, int* out_lines) {
  if (!out_cols || !out_lines) return -1;
  *out_cols = 0;
  *out_lines = 0;
  FILE* fp = fopen(RES_FILE, "r");
  if (!fp) return -1;
  int cols = 0, lines = 0;
  if (fscanf(fp, "%dx%d", &cols, &lines) != 2 &&
      fscanf(fp, "%ux%u", (unsigned*)&cols, (unsigned*)&lines) != 2) {
    fclose(fp);
    return -1;
  }
  fclose(fp);
  if (cols <= 0 || lines <= 0) return -1;
  *out_cols = cols;
  *out_lines = lines;
  return 0;
}

// ---------------- CLI helpers ----------------
static ChargeIconStyle parse_icon_style(const char* s) {
  if (!s) return CHARGE_ICON_PLUG;
  if (!strcasecmp(s, "plug"))   return CHARGE_ICON_PLUG;
  if (!strcasecmp(s, "plus"))   return CHARGE_ICON_PLUS;
  if (!strcasecmp(s, "check"))  return CHARGE_ICON_CHECK;
  if (!strcasecmp(s, "bolt_h")) return CHARGE_ICON_BOLT_H;
  return CHARGE_ICON_PLUG;
}

// ---------------- Main ----------------
int main(int argc, char** argv) {
  uint8_t i2c_node_address = 1;
  int x = -1;
  int y = -1;
  char line[25] = {0};
  char msg[200] = {0};
  char oled_type[10] = {0};
  int clear_line = -1;
  int clear_all = -1;
  int orientation = -1;
  int inverted = -1;
  int display = -1;
  int font = 0;
  int animate = 0;
  int test_frame = 0;
  unsigned frame_delay_ms = 30;
  int power_meter = 0;
  int ina_bus = 10;
  int ina_addr = 0x43;
  unsigned ina_period_ms = 2000;
  char fields_arg[128] = {0};
  uint8_t fields_mask = default_field_mask();
  int show_batt_icon = 0;
  int invert_charge = 0;
  int batt_x_offset = -1;    // centered by default
  int batt_width_px = 18;    // 14..24
  int flash_icon = 0;
  ChargeIconStyle icon_style = CHARGE_ICON_PLUG;
  int video_monitor = 0;
  char video_devs[256] = {0};
  unsigned video_poll_ms = 2000;
  int vid_icon_x_offset = 0; // new: X-offset for video icons
  int cmd_opt = 0;

  while (cmd_opt != -1) {
    cmd_opt = getopt(argc, argv, "I:c::d:f:hi:l:m:n:r:x:y:AS:T::PB:G:U:E:bk:w:zFC:HV:Q:j:");
    switch (cmd_opt) {
      case 'I':
        snprintf(oled_type, sizeof(oled_type) - 1, "%s", optarg);
        break;
      case 'c':
        if (optarg) clear_line = atoi(optarg);
        else clear_all = 1;
        break;
      case 'd':
        display = atoi(optarg);
        break;
      case 'f':
        font = atoi(optarg);
        break;
      case 'h':
        print_help();
        return 0;
      case 'i':
        inverted = atoi(optarg);
        break;
      case 'l':
        strncpy(line, optarg, sizeof(line) - 1);
        line[sizeof(line) - 1] = '\0';
        break;
      case 'm':
        strncpy(msg, optarg, sizeof(msg) - 1);
        msg[sizeof(msg) - 1] = '\0';
        break;
      case 'n':
        i2c_node_address = (uint8_t)atoi(optarg);
        break;
      case 'r':
        orientation = atoi(optarg);
        if (orientation != 0 && orientation != 180) {
          printf("orientation value must be 0 or 180\n");
          return 1;
        }
        break;
      case 'x':
        x = atoi(optarg);
        break;
      case 'y':
        y = atoi(optarg);
        break;
      case 'A':
        animate = 1;
        break;
      case 'T':
        test_frame = 1;
        break;
      case 'P':
        power_meter = 1;
        break;
      case 'B':
        ina_bus = atoi(optarg);
        break;
      case 'G':
        if (strncasecmp(optarg, "0x", 2) == 0) ina_addr = (int)strtol(optarg, NULL, 16);
        else ina_addr = atoi(optarg);
        if (ina_addr < 0) ina_addr = 0x43;
        break;
      case 'U':
        ina_period_ms = (unsigned)atoi(optarg);
        if (ina_period_ms < 50) ina_period_ms = 50;
        break;
      case 'S':
        frame_delay_ms = (unsigned)atoi(optarg);
        if (frame_delay_ms == 0) frame_delay_ms = 1;
        break;
      case 'E':
        strncpy(fields_arg, optarg, sizeof(fields_arg) - 1);
        fields_arg[sizeof(fields_arg) - 1] = '\0';
        fields_mask = parse_enabled_fields(fields_arg);
        break;
      case 'b':
        show_batt_icon = 1;
        break;
      case 'z':
        invert_charge = 1;
        break;
      case 'k':
        batt_x_offset = atoi(optarg);
        if (batt_x_offset < 0) batt_x_offset = -1;
        break;
      case 'w':
        batt_width_px = atoi(optarg);
        if (batt_width_px < 14) batt_width_px = 14;
        if (batt_width_px > 24) batt_width_px = 24;
        break;
      case 'F':
        flash_icon = 1;
        break;
      case 'C':
        icon_style = parse_icon_style(optarg);
        break;
      case 'H':
        video_monitor = 1;
        break;
      case 'V':
        strncpy(video_devs, optarg, sizeof(video_devs) - 1);
        video_devs[sizeof(video_devs) - 1] = '\0';
        break;
      case 'Q':
        video_poll_ms = (unsigned)atoi(optarg);
        if (video_poll_ms < 100) video_poll_ms = 100;
        break;
      case 'j':
        vid_icon_x_offset = atoi(optarg);
        if (vid_icon_x_offset < 0) vid_icon_x_offset = 0;
        break;
      case -1:
        break;
      case '?':
        if (optopt == 'I') {
          printf("prams -%c missing oled type (128x64/128x32/64x48)\n", optopt);
          return 1;
        } else if (optopt == 'd' || optopt == 'f' || optopt == 'i') {
          printf("prams -%c missing 0 or 1 fields\n", optopt);
          return 1;
        } else if (optopt == 'l' || optopt == 'm') {
          printf("prams -%c missing string\n", optopt);
          return 1;
        } else if (optopt == 'n') {
          printf("prams -%c missing 0,1,2... I2C device node number\n", optopt);
          return 1;
        } else if (optopt == 'r') {
          printf("prams -%c missing 0 or 180 fields\n", optopt);
          return 1;
        } else if (optopt == 'x' || optopt == 'y') {
          printf("prams -%c missing coordinate values\n", optopt);
          return 1;
        } else if (optopt == 'S') {
          printf("prams -%c missing ms value\n", optopt);
          return 1;
        } else if (optopt == 'B') {
          printf("prams -%c missing I2C bus number\n", optopt);
          return 1;
        } else if (optopt == 'G') {
          printf("prams -%c missing I2C address (e.g., 0x43)\n", optopt);
          return 1;
        } else if (optopt == 'U') {
          printf("prams -%c missing ms value\n", optopt);
          return 1;
        } else if (optopt == 'E') {
          printf("prams -%c missing fields list (e.g., psu,load,curr,power,pct,vid,fmt|none)\n", optopt);
          return 1;
        } else if (optopt == 'k') {
          printf("prams -%c missing pixel offset\n", optopt);
          return 1;
        } else if (optopt == 'w') {
          printf("prams -%c missing width (14..24)\n", optopt);
          return 1;
        } else if (optopt == 'C') {
          printf("prams -%c missing style (plug|plus|check|bolt_h)\n", optopt);
          return 1;
        } else if (optopt == 'V') {
          printf("prams -%c missing device list (e.g., /dev/video0,/dev/video1)\n", optopt);
          return 1;
        } else if (optopt == 'Q') {
          printf("prams -%c missing ms value\n", optopt);
          return 1;
        } else if (optopt == 'j') {
          printf("prams -%c missing pixel offset\n", optopt);
          return 1;
        } else {
          print_help();
          return 1;
        }
      default:
        print_help();
        return 1;
    }
  }

  // Signals
  signal(SIGINT, on_sigint);
  signal(SIGTERM, on_sigint);

  // Init SSD1306
  uint8_t rc = ssd1306_init(i2c_node_address);
  if (rc != 0) {
    printf("no oled attached to /dev/i2c-%d\n", i2c_node_address);
    return 1;
  }

  int oled_cols = 0, oled_lines = 0;

  // Init OLED module or load persisted resolution
  if (oled_type[0] != 0) {
    if (strcmp(oled_type, "128x64") == 0) {
      rc += ssd1306_oled_default_config(64, 128);
      oled_cols = 128; oled_lines = 64;
    } else if (strcmp(oled_type, "128x32") == 0) {
      rc += ssd1306_oled_default_config(32, 128);
      oled_cols = 128; oled_lines = 32;
    } else if (strcmp(oled_type, "64x48") == 0) {
      rc += ssd1306_oled_default_config(48, 64);
      oled_cols = 64;  oled_lines = 48;
    }
  } else if (ssd1306_oled_load_resolution() != 0) {
    printf("please do init oled module with correction resolution first!\n");
    ssd1306_end();
    return 1;
  } else {
    if (read_saved_resolution(&oled_cols, &oled_lines) != 0) {
      oled_cols = 128;
      oled_lines = 64;
    }
  }

  // Clears, orientation, invert, display power
  if (clear_all > -1) {
    rc += ssd1306_oled_clear_screen();
  } else if (clear_line > -1) {
    rc += ssd1306_oled_clear_line((uint8_t)clear_line);
  }
  if (orientation > -1) rc += ssd1306_oled_set_rotate((uint8_t)orientation);
  if (inverted > -1)    rc += ssd1306_oled_display_flip((uint8_t)inverted);
  if (display > -1)     rc += ssd1306_oled_onoff((uint8_t)display);
  if (x > -1 && y > -1)      rc += ssd1306_oled_set_XY((uint8_t)x, (uint8_t)y);
  else if (x > -1)           rc += ssd1306_oled_set_X((uint8_t)x);
  else if (y > -1)           rc += ssd1306_oled_set_Y((uint8_t)y);
  if (msg[0] != 0) rc += ssd1306_oled_write_string((uint8_t)font, msg);
  else if (line[0] != 0) rc += ssd1306_oled_write_line((uint8_t)font, line);

  // Mode exclusivity
  if ((power_meter + animate + test_frame + video_monitor) > 1) {
    fprintf(stderr, "Note: -A/-T/-P/-H are exclusive. Please select only one.\n");
  }

  // Animation loop (Ctrl-C to stop)
  if (animate) {
    ssd1306_oled_onoff(1);
    if (oled_cols <= 0 || oled_cols > 128) oled_cols = 128;
    if (oled_lines != 32 && oled_lines != 64 && oled_lines != 48) oled_lines = 64;
    run_animation_loop(oled_cols, oled_lines, frame_delay_ms);
  }

  // Test frame (blit from 0/1 image)
  if (test_frame) {
    ssd1306_blit_from_01(image_data, 128, 64);
  }

  // Power meter (with optional video overlays)
  if (power_meter) {
    ssd1306_oled_onoff(1);
    run_power_meter_loop(ina_bus, (uint8_t)ina_addr, ina_period_ms,
                         oled_cols, oled_lines,
                         fields_mask, show_batt_icon, invert_charge,
                         batt_x_offset, batt_width_px,
                         flash_icon, icon_style,
                         video_devs, video_poll_ms,
                         vid_icon_x_offset);
  }

  // Video-only monitor
  if (video_monitor) {
    ssd1306_oled_onoff(1);
    run_video_monitor_loop(oled_cols, oled_lines, video_devs, video_poll_ms, fields_mask, vid_icon_x_offset);
  }

  ssd1306_end();
  return rc;
}
