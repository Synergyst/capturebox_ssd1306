#!/bin/bash

cd /root/ssd1306_linux

/root/ssd1306_linux/ssd1306_bin -n 1 -I 128x64
/root/ssd1306_linux/ssd1306_bin -n 1 -c
#/root/ssd1306_linux/ssd1306_bin -n 1 -P -B 10 -G 0x43 -U 250 -r 180
#/root/ssd1306_linux/ssd1306_bin -P -b -f 0 -E psu,pct,curr -k 96 -w 20 -F -U 1750
#/root/ssd1306_linux/ssd1306_bin -P -b -f 0 -E pct,curr -k 96 -w 20 -F -U 750
#/root/ssd1306_linux/ssd1306_bin -P -b -f 0 -E none -k 0 -w 20 -F -U 750
#/root/ssd1306_linux/ssd1306_bin -c && /root/ssd1306_linux/ssd1306_bin -P -b -f 0 -E vid,fmt -k 0 -w 20 -F -U 750 -V /dev/video0,/dev/video1 -Q 10000 -H -j 21
/root/ssd1306_linux/ssd1306_bin -c && /root/ssd1306_linux/ssd1306_bin -P -b -f 0 -E vid,fmt -k 0 -w 20 -F -U 750 -V /dev/video0,/dev/video1 -Q 10000 -j 21
