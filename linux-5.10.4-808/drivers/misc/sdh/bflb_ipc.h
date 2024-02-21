#ifndef __BFLB_IPC_H__
#define __BFLB_IPC_H__

#include <linux/kernel.h>

/* 0x0 : Interrupt Set Write Register */
#define IPC_CPU1_IPC_ISWR_OFFSET (0x0)
#define IPC_CPU1_IPC_ISWR        IPC_CPU1_IPC_ISWR
#define IPC_CPU1_IPC_ISWR_POS    (0U)
#define IPC_CPU1_IPC_ISWR_LEN    (16U)
#define IPC_CPU1_IPC_ISWR_MSK    (((1U << IPC_CPU1_IPC_ISWR_LEN) - 1) << IPC_CPU1_IPC_ISWR_POS)
#define IPC_CPU1_IPC_ISWR_UMSK   (~(((1U << IPC_CPU1_IPC_ISWR_LEN) - 1) << IPC_CPU1_IPC_ISWR_POS))

/* 0x4 : Interrupt raw status Register */
#define IPC_CPU1_IPC_IRSRR_OFFSET (0x4)
#define IPC_CPU1_IPC_IRSRR        IPC_CPU1_IPC_IRSRR
#define IPC_CPU1_IPC_IRSRR_POS    (0U)
#define IPC_CPU1_IPC_IRSRR_LEN    (16U)
#define IPC_CPU1_IPC_IRSRR_MSK    (((1U << IPC_CPU1_IPC_IRSRR_LEN) - 1) << IPC_CPU1_IPC_IRSRR_POS)
#define IPC_CPU1_IPC_IRSRR_UMSK   (~(((1U << IPC_CPU1_IPC_IRSRR_LEN) - 1) << IPC_CPU1_IPC_IRSRR_POS))

/* 0x8 : Interrupt Clear Register */
#define IPC_CPU1_IPC_ICR_OFFSET (0x8)
#define IPC_CPU1_IPC_ICR        IPC_CPU1_IPC_ICR
#define IPC_CPU1_IPC_ICR_POS    (0U)
#define IPC_CPU1_IPC_ICR_LEN    (16U)
#define IPC_CPU1_IPC_ICR_MSK    (((1U << IPC_CPU1_IPC_ICR_LEN) - 1) << IPC_CPU1_IPC_ICR_POS)
#define IPC_CPU1_IPC_ICR_UMSK   (~(((1U << IPC_CPU1_IPC_ICR_LEN) - 1) << IPC_CPU1_IPC_ICR_POS))

/* 0xc : Interrupt Unmask Set Register */
#define IPC_CPU1_IPC_IUSR_OFFSET (0xc)
#define IPC_CPU1_IPC_IUSR        IPC_AP_IPC_IUSR
#define IPC_CPU1_IPC_IUSR_POS    (0U)
#define IPC_CPU1_IPC_IUSR_LEN    (16U)
#define IPC_CPU1_IPC_IUSR_MSK    (((1U << IPC_CPU1_IPC_IUSR_LEN) - 1) << IPC_CPU1_IPC_IUSR_POS)
#define IPC_CPU1_IPC_IUSR_UMSK   (~(((1U << IPC_CPU1_IPC_IUSR_LEN) - 1) << IPC_CPU1_IPC_IUSR_POS))

/* 0x10 : Interrupt Unmask Clear Register */
#define IPC_CPU1_IPC_IUCR_OFFSET (0x10)
#define IPC_CPU1_IPC_IUCR        IPC_CPU1_IPC_IUCR
#define IPC_CPU1_IPC_IUCR_POS    (0U)
#define IPC_CPU1_IPC_IUCR_LEN    (16U)
#define IPC_CPU1_IPC_IUCR_MSK    (((1U << IPC_CPU1_IPC_IUCR_LEN) - 1) << IPC_CPU1_IPC_IUCR_POS)
#define IPC_CPU1_IPC_IUCR_UMSK   (~(((1U << IPC_CPU1_IPC_IUCR_LEN) - 1) << IPC_CPU1_IPC_IUCR_POS))

/* 0x14 : Interrupt Line Sel Low Register */
#define IPC_CPU1_IPC_ILSLR_OFFSET (0x14)
#define IPC_CPU1_IPC_ILSLR        IPC_CPU1_IPC_ILSLR
#define IPC_CPU1_IPC_ILSLR_POS    (0U)
#define IPC_CPU1_IPC_ILSLR_LEN    (32U)
#define IPC_CPU1_IPC_ILSLR_MSK    (((1U << IPC_CPU1_IPC_ILSLR_LEN) - 1) << IPC_CPU1_IPC_ILSLR_POS)
#define IPC_CPU1_IPC_ILSLR_UMSK   (~(((1U << IPC_CPU1_IPC_ILSLR_LEN) - 1) << IPC_CPU1_IPC_ILSLR_POS))

/* 0x18 : Interrupt Line Sel High Register */
#define IPC_CPU1_IPC_ILSHR_OFFSET (0x18)
#define IPC_CPU1_IPC_ILSHR        IPC_CPU1_IPC_ILSHR
#define IPC_CPU1_IPC_ILSHR_POS    (0U)
#define IPC_CPU1_IPC_ILSHR_LEN    (32U)
#define IPC_CPU1_IPC_ILSHR_MSK    (((1U << IPC_CPU1_IPC_ILSHR_LEN) - 1) << IPC_CPU1_IPC_ILSHR_POS)
#define IPC_CPU1_IPC_ILSHR_UMSK   (~(((1U << IPC_CPU1_IPC_ILSHR_LEN) - 1) << IPC_CPU1_IPC_ILSHR_POS))

/* 0x1C : Interrupt status Register */
#define IPC_CPU1_IPC_ISR_OFFSET (0x1C)
#define IPC_CPU1_IPC_ISR        IPC_CPU1_IPC_ISR
#define IPC_CPU1_IPC_ISR_POS    (0U)
#define IPC_CPU1_IPC_ISR_LEN    (16U)
#define IPC_CPU1_IPC_ISR_MSK    (((1U << IPC_CPU1_IPC_ISR_LEN) - 1) << IPC_CPU1_IPC_ISR_POS)
#define IPC_CPU1_IPC_ISR_UMSK   (~(((1U << IPC_CPU1_IPC_ISR_LEN) - 1) << IPC_CPU1_IPC_ISR_POS))

/* 0x20 : Interrupt Set Write Register */
#define IPC_CPU0_IPC_ISWR_OFFSET (0x20)
#define IPC_CPU0_IPC_ISWR        IPC_CPU0_IPC_ISWR
#define IPC_CPU0_IPC_ISWR_POS    (0U)
#define IPC_CPU0_IPC_ISWR_LEN    (16U)
#define IPC_CPU0_IPC_ISWR_MSK    (((1U << IPC_CPU0_IPC_ISWR_LEN) - 1) << IPC_CPU0_IPC_ISWR_POS)
#define IPC_CPU0_IPC_ISWR_UMSK   (~(((1U << IPC_CPU0_IPC_ISWR_LEN) - 1) << IPC_CPU0_IPC_ISWR_POS))

/* 0x24 : Interrupt raw status Register */
#define IPC_CPU0_IPC_IRSRR_OFFSET (0x24)
#define IPC_CPU0_IPC_IRSRR        IPC_CPU0_IPC_IRSRR
#define IPC_CPU0_IPC_IRSRR_POS    (0U)
#define IPC_CPU0_IPC_IRSRR_LEN    (16U)
#define IPC_CPU0_IPC_IRSRR_MSK    (((1U << IPC_CPU0_IPC_IRSRR_LEN) - 1) << IPC_CPU0_IPC_IRSRR_POS)
#define IPC_CPU0_IPC_IRSRR_UMSK   (~(((1U << IPC_CPU0_IPC_IRSRR_LEN) - 1) << IPC_CPU0_IPC_IRSRR_POS))

/* 0x28 : Interrupt Clear Register */
#define IPC_CPU0_IPC_ICR_OFFSET (0x28)
#define IPC_CPU0_IPC_ICR        IPC_CPU0_IPC_ICR
#define IPC_CPU0_IPC_ICR_POS    (0U)
#define IPC_CPU0_IPC_ICR_LEN    (16U)
#define IPC_CPU0_IPC_ICR_MSK    (((1U << IPC_CPU0_IPC_ICR_LEN) - 1) << IPC_CPU0_IPC_ICR_POS)
#define IPC_CPU0_IPC_ICR_UMSK   (~(((1U << IPC_CPU0_IPC_ICR_LEN) - 1) << IPC_CPU0_IPC_ICR_POS))

/* 0x2c : Interrupt Unmask Set Register */
#define IPC_CPU0_IPC_IUSR_OFFSET (0x2c)
#define IPC_CPU0_IPC_IUSR        IPC_CPU0_IPC_IUSR
#define IPC_CPU0_IPC_IUSR_POS    (0U)
#define IPC_NP_IPC_IUSR_LEN      (16U)
#define IPC_CPU0_IPC_IUSR_MSK    (((1U << IPC_CPU0_IPC_IUSR_LEN) - 1) << IPC_CPU0_IPC_IUSR_POS)
#define IPC_CPU0_IPC_IUSR_UMSK   (~(((1U << IPC_CPU0_IPC_IUSR_LEN) - 1) << IPC_CPU0_IPC_IUSR_POS))

/* 0x30 : Interrupt Unmask Clear Register */
#define IPC_CPU0_IPC_IUCR_OFFSET (0x30)
#define IPC_CPU0_IPC_IUCR        IPC_CPU0_IPC_IUCR
#define IPC_CPU0_IPC_IUCR_POS    (0U)
#define IPC_CPU0_IPC_IUCR_LEN    (16U)
#define IPC_CPU0_IPC_IUCR_MSK    (((1U << IPC_CPU0_IPC_IUCR_LEN) - 1) << IPC_CPU0_IPC_IUCR_POS)
#define IPC_CPU0_IPC_IUCR_UMSK   (~(((1U << IPC_CPU0_IPC_IUCR_LEN) - 1) << IPC_CPU0_IPC_IUCR_POS))

/* 0x34 : Interrupt Line Sel Low Register */
#define IPC_CPU0_IPC_ILSLR_OFFSET (0x34)
#define IPC_CPU0_IPC_ILSLR        IPC_CPU0_IPC_ILSLR
#define IPC_CPU0_IPC_ILSLR_POS    (0U)
#define IPC_CPU0_IPC_ILSLR_LEN    (32U)
#define IPC_CPU0_IPC_ILSLR_MSK    (((1U << IPC_CPU0_IPC_ILSLR_LEN) - 1) << IPC_CPU0_IPC_ILSLR_POS)
#define IPC_CPU0_IPC_ILSLR_UMSK   (~(((1U << IPC_CPU0_IPC_ILSLR_LEN) - 1) << IPC_CPU0_IPC_ILSLR_POS))

/* 0x38 : Interrupt Line Sel High Register */
#define IPC_CPU0_IPC_ILSHR_OFFSET (0x38)
#define IPC_CPU0_IPC_ILSHR        IPC_CPU0_IPC_ILSHR
#define IPC_CPU0_IPC_ILSHR_POS    (0U)
#define IPC_CPU0_IPC_ILSHR_LEN    (32U)
#define IPC_CPU0_IPC_ILSHR_MSK    (((1U << IPC_CPU0_IPC_ILSHR_LEN) - 1) << IPC_CPU0_IPC_ILSHR_POS)
#define IPC_CPU0_IPC_ILSHR_UMSK   (~(((1U << IPC_CPU0_IPC_ILSHR_LEN) - 1) << IPC_CPU0_IPC_ILSHR_POS))

/* 0x3C : Interrupt status Register */
#define IPC_CPU0_IPC_ISR_OFFSET (0x3C)
#define IPC_CPU0_IPC_ISR        IPC_CPU0_IPC_ISR
#define IPC_CPU0_IPC_ISR_POS    (0U)
#define IPC_CPU0_IPC_ISR_LEN    (16U)
#define IPC_CPU0_IPC_ISR_MSK    (((1U << IPC_CPU0_IPC_ISR_LEN) - 1) << IPC_CPU0_IPC_ISR_POS)
#define IPC_CPU0_IPC_ISR_UMSK   (~(((1U << IPC_CPU0_IPC_ISR_LEN) - 1) << IPC_CPU0_IPC_ISR_POS))

struct ipc_port {
	spinlock_t		lock;			/* port lock */
	unsigned long		iobase;			/* in/out[bwl] */
	unsigned char __iomem	*membase;		/* read/write[bwl] */
	unsigned int            irq;                    /* irq number */
	struct device           *dev;                   /* parent device */
	resource_size_t		mapbase;		/* for ioremap */
	resource_size_t		mapsize;
};
#endif /* __IPC_REG_H__ */
