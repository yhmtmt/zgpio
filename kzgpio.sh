#!/bin/sh

module="zgpio"
device="zgpio"

/sbin/rmmod $module.ko $* || exit 1

rm -f /dev/${device}0
 

 