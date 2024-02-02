#! /bin/sh
insmod /lib/modules/xram_net.ko
ifconfig bleth0 192.168.10.99
telnetd
