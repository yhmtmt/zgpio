#!/bin/sh

module="zgpio"
device="zgpio"

/sbin/rmmod $module $* || exit 1

rm -f /dev/${device}*
 

 
