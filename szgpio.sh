#!/bin/sh

module="zgpio"
device="zgpio"
mode="664"

/sbin/insmod ./$module.ko $* || exit 1


majors=$(awk "\$2==\"$module\" {print \$1}" /proc/devices)

i=0

for major in $majors
do
    rm -f /dev/${device}$i
    mknod /dev/${device}$i c $major 0

    group="staff"
    grep -q '^staff:' /etc/group || group="root"

    chgrp $group /dev/${device}$i
    chmod $mode /dev/${device}$i
    i=`expr $i + 1`
done
