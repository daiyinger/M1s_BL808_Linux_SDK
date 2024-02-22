#include <linux/module.h> 
#include <linux/blkdev.h> 
#include <linux/hdreg.h> 
#include <linux/version.h>
#include <linux/vmalloc.h>

/*
*     insmod sdblkdev.ko 
*     # or insmod sdblkdev.ko size=numK/M/G/T 
*     fdisk /dev/sdblkdev # create 2 patitions 
*     mkfs.ext2 /dev/sdblkdev1 
*     mkfs.ext2 /dev/sdblkdev2 
*     mount /dev/sdblkdev1 /mnt/temp1/ 
*     mount /dev/sdblkdev2 /mnt/temp2/ 
*     # play in /mnt/temp1/ and /mnt/temp2/ 
*     umount /mnt/temp1/ 
*     umount /mnt/temp2/ 
*     rmmod sdblkdev.ko 
* 
*/
static int sd_block_major=0;
static struct request_queue *sdblkdev_queue; 
static struct gendisk *sdblkdev_disk;

#define TINY4412_BLK_DEV_BYTES        (1024*1024*4)  /*设置块设备的大小*/
static unsigned char *sizeof_p;

/*
* Handle an I/O request.
* 实现扇区的读写

unsigned long sector:  当前扇区位置
unsigned long nsect :  扇区读写数量
char *buffer        :  读写的缓冲区指针
int write           :  是读还是写
*/
int SDH_ReadMultiBlocks(uint8_t *readbuff, uint32_t ReadAddr, uint16_t BlockSize, uint32_t NumberOfBlocks);
int SDH_WriteMultiBlocks(uint8_t *writebuff, uint32_t WriteAddr, uint16_t BlockSize, uint32_t NumberOfBlocks);
static void sd_block_dev_sector_read_write(unsigned long sector,unsigned long nsect, char *buffer, int write)
{
		/*块设备最小单位是一个扇区，一个扇区的字节数是512字节*/
		unsigned long offset = sector;  /*写入数据的位置*/
		unsigned long nbytes = nsect;   /*写入的长度*/
		if((offset + nbytes)>/*TINY4412_BLK_DEV_BYTES*/122507264UL*512)
		{
			printk("Over(%ld %ld)\n", offset, nbytes);
			return;
		}
		if(write) /*为真,表示是写*/
		{
			//memcpy(sizeof_p + offset, buffer, nbytes);
			
			int page = nbytes>>9;
			int i;
			for(i = 0; i< page; i++)
			{
				SDH_WriteMultiBlocks(buffer+512*i, 
					 offset+512*i, 512, 1);
			}
		}
		else      /*读操作*/
		{
			//memcpy(buffer,sizeof_p + offset, nbytes);
#if 1
			//printk("offset %ld nbytes:%ld page:%ld\n", offset>>9, nbytes, nbytes>>9);
			int page = nbytes>>9;
			int i;
			for(i = 0; i< page; i++)
			{
				SDH_ReadMultiBlocks(buffer+512*i, 
					 offset+512*i, 512, 1);
			}
#else
			SDH_ReadMultiBlocks(buffer,offset, 512, nbytes>>9);
#endif
#if 0
			char strbuf[1024] = {0};
			for(i = 1; i <= nbytes; i++)
			{
				sprintf(strbuf+strlen(strbuf), "%02x ", buffer[i-1]);
				if(i%20==0)
				{
					printk("%s\n", strbuf);
					strbuf[0] = 0;
				}
			}
			printk("%s\n", strbuf);
#endif
		}
}

/*
处理请求
*/
static blk_qc_t sdblkdev_make_request(/*struct request_queue *q, */struct bio *bio) 
{ 
	int dir; 
	unsigned long long dsk_offset; 
	struct bio_vec bvec;
       	struct bvec_iter iter;	
	int i; 
	void *iovec_mem;
	
	/*判断读写方向*/
	if(bio_data_dir(bio) == WRITE) 
		dir = 1;
	else 
		dir = 0;
	dsk_offset = bio->bi_iter.bi_sector << 9;
	bio_for_each_segment(bvec, bio, iter) 
	{ 
		iovec_mem = kmap(bvec.bv_page) + bvec.bv_offset; 

		//printk("iovec_mem:%px dir:%d dsk_offset:%lld\n", iovec_mem, 
		//		dir, dsk_offset);
		
		//起始位置,长度,源数据,方向
		sd_block_dev_sector_read_write(dsk_offset,bvec.bv_len,iovec_mem,dir);
		
		kunmap(bvec.bv_page);
		dsk_offset += bvec.bv_len; 
	}
	bio_endio(bio);
        return BLK_QC_T_NONE;	
}
struct block_device_operations sdblkdev_fops = 
{ 
    .owner= THIS_MODULE,
    .submit_bio	= sdblkdev_make_request, 
};
static int __init sdblkdev_init(void) 
{ 
	sizeof_p=vmalloc(TINY4412_BLK_DEV_BYTES);
	if(sizeof_p==NULL)
	{
		printk("空间申请失败!\n");
		return 0;
	}
	/*动态分配请求队列*/
	sdblkdev_queue = blk_alloc_queue(GFP_KERNEL);
	
	/*绑定请求队列*/
	//blk_queue_make_request(sdblkdev_queue, 
	//		sdblkdev_make_request);
	
	/*动态分配次设备号结构*/
	/*每一个磁盘(分区)都是使用一个gendisk结构保存*/
	sdblkdev_disk = alloc_disk(64); 
	
	/*磁盘名称赋值*/
	strcpy(sdblkdev_disk->disk_name, "sdblkdev"); 

	/*注册一个块设备,自动分配主设备号*/
	sd_block_major = register_blkdev(0,"sd_block");
	printk("sd_block_major=%d\n",sd_block_major);
	
	sdblkdev_disk->major=sd_block_major; 	  /*主设备号*/
	sdblkdev_disk->first_minor = 0; 				  /*次设备号*/
	sdblkdev_disk->fops = &sdblkdev_fops;   /*文件操作结合*/
	sdblkdev_disk->queue = sdblkdev_queue;  /*处理数据请求的队列*/
	
	/*设置磁盘结构 capacity 的容量*/
	/*注意: 块设备的大小使用扇区作为单位设置，而扇区的大小默认是512字节。
	  cat /sys/block/xxxx/size 可以查看到设置的大小
	  把字节为单位的大小转换为以扇区为单位时，我们需要除以512，或者右移9位
	*/
	set_capacity(sdblkdev_disk, 122507264);//TINY4412_BLK_DEV_BYTES>>9); //122507264
	
	//添加磁盘信息到内核
	add_disk(sdblkdev_disk);
	return 0;
}

static void __exit sdblkdev_exit(void) 
{ 
	//删除磁盘
	del_gendisk(sdblkdev_disk);
	
	put_disk(sdblkdev_disk); 
	
	//清除队列
	blk_cleanup_queue(sdblkdev_queue);
	
	/*注销块设备*/
	unregister_blkdev(sd_block_major, "sd_block");
	
	vfree(sizeof_p);
}

module_init(sdblkdev_init); 
module_exit(sdblkdev_exit);
MODULE_LICENSE("GPL");

