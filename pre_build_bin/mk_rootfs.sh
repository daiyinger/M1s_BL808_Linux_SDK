#! /bin/sh
SHELL_DIR=$(cd "$(dirname "$0")"; pwd)
xram_net_path=$SHELL_DIR/../out/modules/lib/modules/5.10.4/build/drivers/net/xram_net/xram_net.ko
if [ -f $xram_net_path ]; then
	echo "cp xram_net.ko ..."
	cp $xram_net_path rootfs/lib/modules/
fi

sdh_path=$SHELL_DIR/../out/modules/lib/modules/5.10.4/build/drivers/misc/sdh/sdh.ko
if [ -f $sdh_path ]; then
	echo "cp sdh.ko ..."
	cp $sdh_path rootfs/lib/modules/
fi


if [ ! -d rootfs/dev ]; then
	mkdir rootfs/dev
	cd rootfs/dev
	sudo mknod -m 660 console c 5 1
	sudo mknod -m 660 null c 1 3
	mkdir pts
	cd -
fi

if [ ! -d rootfs/proc ]; then
	mkdir rootfs/proc
fi

if [ ! -d rootfs/mnt/nfs ]; then
	mkdir -p rootfs/mnt/nfs
fi

if [ ! -d rootfs/mnt/sd ]; then
	mkdir -p rootfs/mnt/sd
fi

genext2fs -b 8192 -d rootfs ramdisk

rm -f  ramdisk.lz4
lz4 ramdisk ramdisk.lz4
