#!/bin/bash

cd /root/ssd1306_linux

make -j6
/root/ssd1306_linux/ssd1306_bin -n 1 -I 128x64
/root/ssd1306_linux/ssd1306_bin -n 1 -c
#/root/ssd1306_linux/ssd1306_bin -n 1 -P -B 10 -G 0x43 -U 250 -r 180
#/root/ssd1306_linux/ssd1306_bin -P -b -f 0 -E psu,pct,curr -k 96 -w 20 -F -U 1750
#/root/ssd1306_linux/ssd1306_bin -P -b -f 0 -E pct,curr -k 96 -w 20 -F -U 750
#/root/ssd1306_linux/ssd1306_bin -P -b -f 0 -E none -k 0 -w 20 -F -U 750
#/root/ssd1306_linux/ssd1306_bin -c && /root/ssd1306_linux/ssd1306_bin -P -b -f 0 -E vid,fmt -k 0 -w 20 -F -U 750 -V /dev/video0,/dev/video1 -Q 10000 -H -j 21
#/root/ssd1306_linux/ssd1306_bin -c && /root/ssd1306_linux/ssd1306_bin -P -b -f 0 -E vid,fmt -k 0 -w 20 -F -U 750 -V /dev/video0,/dev/video1 -Q 375 -j 22
#/root/ssd1306_linux/ssd1306_bin -c && /root/ssd1306_linux/ssd1306_bin -P -b -f 0 -E vid,fmt -k 0 -w 20 -F -U 750 -V /dev/video0 -Q 375 -j 22
#/root/ssd1306_linux/ssd1306_bin -c && /root/ssd1306_linux/ssd1306_bin -P -b -f 0 -E psu,curr,pct -k 0 -w 20 -F -U 750
#/root/ssd1306_linux/ssd1306_bin -c && /root/ssd1306_linux/ssd1306_bin -P -b -f 0 -E local_time,local_date,lat,lon -k 0 -w 20 -F -U 750
/root/ssd1306_linux/ssd1306_bin -c && /root/ssd1306_linux/ssd1306_bin -P -b -f 0 -E utc_time,utc_date,lat,lon -k 0 -w 20 -F -U 750
