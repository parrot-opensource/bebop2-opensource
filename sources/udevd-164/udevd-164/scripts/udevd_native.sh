#!/bin/sh

set -m

SBIN=$(dirname $0)

export LD_LIBRARY_PATH=$(realpath $SBIN/../lib)

[ -x "$SBIN/udevd" ] || exit 1

udev_root=$($SBIN/udevadm info --root)

mkdir -p $udev_root || exit 1

# export board name as a file
echo "JUBA_BOARD=native" > $udev_root/.udevd_board

$SBIN/udevd --debug &

$SBIN/udevadm trigger --action=add \
    --subsystem-match=hidraw \
    --subsystem-match=mmc \
    --subsystem-match=usb \
    --subsystem-match=tty \
    --subsystem-match=sound \
    --subsystem-match=firmware \
    --subsystem-match=block

$SBIN/udevadm settle

cat /sys/kernel/uevent_seqnum > $udev_root/.udevd_cold_seqnum

fg
