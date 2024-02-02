#! /bin/sh
SHELL_DIR=$(cd "$(dirname "$0")"; pwd)
xram_net_path=$SHELL_DIR/../out/modules/lib/modules/5.10.4/build/drivers/net/xram_net/xram_net.ko
if [ -f $xram_net_path ]; then
	echo "cp xram_net.ko ..."
	cp $xram_net_path rootfs/lib/modules/
fi

genext2fs -b 8192 -d rootfs ramdisk

rm -f  ramdisk.lz4
lz4 ramdisk ramdisk.lz4
