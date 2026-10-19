#!/bin/bash
MEDIADEV=$(v4l2-ctl --list-devices | grep rp1-cfe -A10 | grep media | sed 's/\t//')
media-ctl -d $MEDIADEV --set-v4l2 "'ap1302.6-003c':2 [fmt:UYVY8_1X16/1280x480 crop:(0,0)/1280x480]"
echo "running postconfig.sh" | systemd-cat
