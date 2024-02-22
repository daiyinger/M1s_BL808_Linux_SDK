import os
import sys

compress_flag = 0

def create_flash_default_data(length):
    datas = bytearray(length)
    for i in range(length):
        datas[i] = 0xff
    return datas

def gen_flash_bin():
    rootfs_base = 0x500000
    bootload_file = "../pre_build_bin/spl_bl808_d0.bin"
    dtb_file = "hw.dtb.5M"                  # 64k
    opensbi_file = "fw_jump.bin" #"../pre_build_bin/opensbi_v0.6.bin" # 64k
    kernel_file = "Image.lz4"

    kernel_out_img_file = "./whole_img_d0.bin"
    linux_rootfs_file = "ramdisk.lz4"
    bootload_file_size = os.stat(bootload_file).st_size
    dtb_file_size = os.stat(dtb_file).st_size
    linux_opensbi_file_size = os.stat(opensbi_file).st_size
    kernel_file_size = os.stat(kernel_file).st_size
    rootfs_file_size = os.stat(linux_rootfs_file).st_size

    whole_img_data = create_flash_default_data(rootfs_base + rootfs_file_size)

    # bootload
    print("bootload size:", bootload_file_size)
    fp = open(bootload_file, 'rb')
    data00 = fp.read() + bytearray(0)
    fp.close()
    whole_img_data[0x0:0x0+len(data00)] = data00  # 0x0~0x10000 64k    

    # dtb
    print("dtb size:", dtb_file_size)
    fp = open(dtb_file, 'rb')
    data0 = fp.read() + bytearray(0)
    fp.close()
    whole_img_data[0x10000:0x10000+len(data0)] = data0  # 0x0~0x10000 64k

    # opensbi
    print("opensbi size:",linux_opensbi_file_size)
    fp = open(opensbi_file, 'rb')
    data1 = fp.read() + bytearray(0)
    fp.close()
    whole_img_data[0x20000:0x20000+len(data1)] = data1  # 0x10000~0x20000 64k

    # linux kernel
    print("kernel img size:",kernel_file_size)

    b0 = (kernel_file_size & 0xff000000) >> 24
    b1 = (kernel_file_size & 0xff0000) >> 16
    b2 = (kernel_file_size & 0xff00) >> 8
    b3 = kernel_file_size & 0xff
    # print(b0)
    # print(b1)
    # print(b2)
    # print(b3)
    header2 = [0x00,0x00,0x00,compress_flag,b3,b2,b1,b0]
    whole_img_data[0x2fff8:0x30000] = bytearray((header2)) # image header
    fp = open(kernel_file, 'rb')
    data2 = fp.read() + bytearray(0)
    fp.close()
    whole_img_data[0x30000:0x30000+len(data2)] = data2 # 4M

    #opensbi_linux_v0.6.bin
    '''
    fp = open(linux_opensbi, 'rb')
    data21 = fp.read() + bytearray(0)
    fp.close()
    whole_img_data[0x100000-0x10000:0x100000-0x10000+len(data21)] = data21 #linux kernel

    fp = open(linux_kernel_file, 'rb')
    data3 = fp.read() + bytearray(0)
    fp.close()
    whole_img_data[0x100000:0x100000+len(data3)] = data3 #linux kernel
    '''
    linux_rootfs_file_size = os.stat(linux_rootfs_file).st_size
    print("rootfs size:",linux_rootfs_file_size)
    fp = open(linux_rootfs_file, 'rb')
    data3 = fp.read() + bytearray(0)
    fp.close()


    b0 = (linux_rootfs_file_size & 0xff000000) >> 24
    b1 = (linux_rootfs_file_size & 0xff0000) >> 16
    b2 = (linux_rootfs_file_size & 0xff00) >> 8
    b3 = linux_rootfs_file_size & 0xff
    header3 = [b3,b2,b1,b0]

    #rootfs header
    whole_img_data[rootfs_base-4:rootfs_base] = bytearray((header3))
    whole_img_data[rootfs_base:rootfs_base+len(data3)] = data3

    print("kernel_out_img_file size", len(whole_img_data))

    fp = open(kernel_out_img_file, 'wb+')
    fp.write(whole_img_data)
    fp.close()

if __name__ == '__main__':
    compress_flag = 1
    print("merge bin start...")
    gen_flash_bin()
    print("merge done!")

    
