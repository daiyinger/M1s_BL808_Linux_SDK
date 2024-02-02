# SPDX-License-Identifier: GPL-2.0-only
ccflags-y += -D BL_INTF_XRAM
obj-m		+= xram_net.o
xram_net-objs := bl_xram_eth.o
xram_net-objs += xram_msg_handlers.o
xram_net-objs += main.o
xram_net-objs += wifi.o
xram_net-objs += msg_handlers.o
xram_net-objs += ctl_port.o
