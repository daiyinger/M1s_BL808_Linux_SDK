#ifndef __SDH_H__
#define __SDH_H__
#include <linux/module.h>  
#include <linux/kernel.h>  
#include <linux/fs.h>  
#include <asm/io.h>  
#include <linux/cdev.h>  
#include <linux/device.h>  
#include <linux/dma-mapping.h>  
#include <linux/interrupt.h> 
#include <linux/irqdomain.h>
#include <linux/random.h>
#include <linux/blkdev.h> 
#include <linux/hdreg.h> 
#include <linux/version.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>

//#include "core_rv64.h"
#include "ipc_reg.h"
#include "sdh_reg.h"
#include "bl_sdh.h"
#define XRAM_ADDRESS 0x40000000

#define PHYSICAL_ADDRESS 0x20060000//SDH_BASE_ADDR //0x3F002000   
#undef SDH_BASE_ADDR
#define SIZE 4096  // 示例大小，你可以根据需要更改  

#define DEVICE_NAME "shm"  
#define DEVICE_FILE_NAME "/dev/" DEVICE_NAME  


inline void __DCACHE_IPA(uint64_t addr)
{
	__asm volatile("dcache.ipa %0"
			:
			: "r"(addr));
}

inline void __DCACHE_CIPA(uint64_t addr)
{
	__asm volatile("dcache.cipa %0"
			:
                        : "r"(addr));
}

inline void __DCACHE_CPA(uint64_t addr)
{
	    __asm volatile("dcache.cpa %0"
		        :
       			: "r"(addr));
}

static inline void csi_dcache_invalid_range(void *addr, int64_t dsize)
{
	int64_t op_size = dsize + (uint64_t)addr % 64;
	uint64_t op_addr = (uint64_t)addr;
	int64_t linesize = 64;

	__asm volatile("fence");
	
	while (op_size > 0) {
		__DCACHE_IPA(op_addr);
		op_addr += linesize;
		op_size -= linesize;
	}

	__asm volatile("fence");
}

static inline void csi_dcache_clean_range(void *addr, int64_t dsize)
{
	int64_t op_size = dsize + (uint64_t)addr % 64;
	uint64_t op_addr = (uint64_t)addr & (0xFFFFFFFFUL << 6U);
	int64_t linesize = 64;

	__asm volatile("fence");
	
	while (op_size > 0) {
		__DCACHE_CPA(op_addr);
		op_addr += linesize;
		op_size -= linesize;
	}

	__asm volatile("fence");
}

inline void csi_dcache_clean_invalid_range(void *addr, int64_t dsize)
{

	int64_t op_size = dsize + (uint64_t)addr % 64;
	uint64_t op_addr = (uint64_t)addr;
	int64_t linesize = 64;

	__asm volatile("fence");

	while (op_size > 0) {
		__DCACHE_CIPA(op_addr);
	    	op_addr += linesize;
		op_size -= linesize;
	}
	
	__asm volatile("fence");
}


inline void csi_l2cache_clean_invalid(void)
{

	__asm volatile("fence");
	__asm volatile("fence.i");	
	__asm volatile("l2cache.ciall");
	__asm volatile("fence");
	__asm volatile("fence.i");	

}

inline void csi_l2cache_clean(void)
{

	__asm volatile("fence");
	__asm volatile("fence.i");	
	__asm volatile("l2cache.call");
	__asm volatile("fence");
	__asm volatile("fence.i");	

}

inline void csi_l2cache_invalid(void)
{

	__asm volatile("fence");
	__asm volatile("fence.i");	
	__asm volatile("l2cache.iall");
	__asm volatile("fence");
	__asm volatile("fence.i");	

}

#define _RT_STRINGIFY(x...)     #x
#define RT_STRINGIFY(x...) _RT_STRINGIFY(x)
#define __OPC_INSN_FORMAT_R(opcode, func3, func7, rd, rs1, rs2) \
	    ".insn r "RT_STRINGIFY(opcode)","RT_STRINGIFY(func3)","RT_STRINGIFY(func7)","RT_STRINGIFY(rd)","RT_STRINGIFY(rs1)","RT_STRINGIFY(rs2)

#define __OPC_INSN_FORMAT_CACHE(func7, rs2, rs1) \
	    __OPC_INSN_FORMAT_R(0x0b, 0x0, func7, x0, rs1, rs2)

#define OPC_DCACHE_IVA(rs1)     __OPC_INSN_FORMAT_CACHE(0x1, x6, rs1)
#define OPC_DCACHE_CVA(rs1)     __OPC_INSN_FORMAT_CACHE(0x1, x4, rs1)

#define CACHE_OP_RS1 %0

#define CACHE_OP_RANGE(instr)                                  \
	{                                                          \
	register uint64_t i = start & ~(L1_CACHE_BYTES - 1); \
	for (; i < end; i += L1_CACHE_BYTES)                   \
	{                                                      \
	__asm__ volatile(instr ::"r"(i)                    \
	: "memory");                      \
	}                                                      \
	}

inline void dcache_inv_range(unsigned long start, unsigned long end)
{
    CACHE_OP_RANGE(OPC_DCACHE_IVA(CACHE_OP_RS1));
}

inline void dcache_wb_range(unsigned long start, unsigned long end)
{
    CACHE_OP_RANGE(OPC_DCACHE_CVA(CACHE_OP_RS1));
}

#define OPC_SYNC                ".long 0x0180000B"
#define hw_cpu_sync() __asm__ volatile(OPC_SYNC:: \
		                                              : "memory")

inline void cpu_dcache_invalidate_local(void *addr, int size)
{
	dcache_inv_range((unsigned long)addr, 
			(unsigned long)((unsigned char *)addr + size));
	hw_cpu_sync();
}

inline void cpu_dcache_clean_local(void *addr, int size)
{
	dcache_wb_range((unsigned long)addr, (unsigned long)((unsigned char *)addr + size));
	hw_cpu_sync();
}


#define BL_WR_WORD(addr, val)  ((*(volatile uint32_t *)(uintptr_t)(addr)) = (val))
#define BL_WR_REG(addr, regname, val)             BL_WR_WORD(addr + regname##_OFFSET, val)

#define BL_RD_WORD(addr)       (*((volatile uint32_t *)(uintptr_t)(addr)))
#define BL_RD_REG(addr, regname)                  BL_RD_WORD(addr + regname##_OFFSET)

#define BL_SET_REG_BITS_VAL(val, bitname, bitval) (((val)&bitname##_UMSK) | ((uint32_t)(bitval) << bitname##_POS))

#define BL_RD_SHORT(addr)      (*((volatile uint16_t *)(uintptr_t)(addr)))

#define BL_WR_SHORT(addr, val) ((*(volatile uint16_t *)(uintptr_t)(addr)) = (val))
#define BL_WR_REG16(addr, regname, val)           BL_WR_SHORT(addr + regname##_OFFSET, val)

#endif
