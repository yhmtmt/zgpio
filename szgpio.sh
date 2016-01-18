#!/bin/sh

module="zgpio"
device="zgpio"
mode="664"

/sbin/insmod ./$module.ko $* || exit 1

rm -f /dev/${device}*

majors=$(awk "\$2==\"$module\" {print \$1}" /proc/devices)


for major in $majors
do
    mknod /dev/${device}0 c $major 0

    group="staff"
    grep -q '^staff:' /etc/group || group="root"

    chgrp $group /dev/${device}0
    chmod $mode /dev/${device}0
done
