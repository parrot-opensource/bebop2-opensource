#!/bin/sh

# start udevd with udev_init launcher
# Note: application can overide susbsystem triggered at startup using shell arguments
#       ex: udevd.sh trigger --action=add --subsystem-match=usb --subsystem-match=firmware

exec udevd_init $@
