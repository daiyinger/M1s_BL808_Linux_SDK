// SPDX-License-Identifier: GPL-2.0+
/*
 * bflb_ipc.c -- BFLB UART driver
 *
 * Based on mcf.c -- Freescale ColdFire UART driver
 *
 */

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/console.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/debugfs.h>
#include <linux/workqueue.h>  
#include "bflb_ipc.h"

struct work_struct ipc_work;  

#define DRV_NAME "bflb_ipc"
#define BFLB_IPC_SIZE 1024

int ipi_cnt = 0;
void *ipc_base = NULL;

#define BL_WR_WORD(addr, val)  ((*(volatile uint32_t *)(uintptr_t)(addr)) = (val))
#define BL_WR_REG(addr, regname, val)             BL_WR_WORD(addr + regname##_OFFSET, val)
#define BL_RD_REG(addr, regname)                  BL_RD_WORD(addr + regname##_OFFSET)
#define BL_RD_WORD(addr)       (*((volatile uint32_t *)(uintptr_t)(addr)))


static void ipc_work_handler(struct work_struct *dwork)  
{  
	printk(KERN_INFO "ipc work handler %d!\n", ipi_cnt);  
	
	//queue_delayed_work(system_wq, &ipc_work, msecs_to_jiffies(1000));       
}


//return __raw_readl(port->membase + reg);

//__raw_writel(value, port->membase + reg);

//return __raw_readb(port->membase + UART_FIFO_RDATA_OFFSET);
//__raw_writeb(value, port->membase + UART_FIFO_WDATA_OFFSET);


static irqreturn_t ipi_interrupt_handler(int irq, void *dev_id)  
{  
	uint32_t irqStatus = BL_RD_REG(ipc_base, IPC_CPU0_IPC_IRSRR);
	//handle
	ipi_cnt++;   
	
	queue_work(system_wq, &ipc_work);
	
	BL_WR_REG(ipc_base, IPC_CPU0_IPC_ICR, irqStatus);
	return IRQ_HANDLED;  
}

static int bflb_ipc_startup(struct ipc_port *port)
{
	unsigned long flags;
	int ret;

	printk("%s port->irq:%d\n", DRV_NAME, port->irq);
	ret = request_irq(port->irq, ipi_interrupt_handler, 0, DRV_NAME, port);
	if (ret) {
		pr_err(DRV_NAME ": unable to attach BFLB IPC"
				"interrupt vector=%d\n",
		       port->irq);
		return ret;
	}

	spin_lock_irqsave(&port->lock, flags);

	//printk("success\n");
	//enable_irq(port->irq);
		
        INIT_WORK(&ipc_work, ipc_work_handler);  
        //queue_delayed_work(system_wq, &ipc_work, msecs_to_jiffies(1000));       
	
	spin_unlock_irqrestore(&port->lock, flags);

	return 0;
}

static struct ipc_port bflb_ipc_port;



static int bflb_ipc_probe(struct platform_device *pdev)
{

	//printk("bflb_ipc_probe!!!!!!!!!!!!!!!!!\r\n");

	//port = &bflb_ipc_port[i].port;
	//bflb_ipc = to_bflb_ipc_port(port);

	bflb_ipc_port.mapbase = pdev->resource[0].start;
	
	bflb_ipc_port.irq = pdev->resource[1].start;

	//printk("port->irq %d port->mapbase:%llx\n", port->irq, port->mapbase);

	bflb_ipc_port.membase = ioremap(bflb_ipc_port.mapbase, BFLB_IPC_SIZE);
	if (!bflb_ipc_port.membase)
		return -ENOMEM;

	ipc_base = bflb_ipc_port.membase;
	//port->line = i;
	//port->type = PORT_BFLB_UART;
	//port->iotype = SERIAL_IO_MEM;
	//port->ops = &bflb_ipc_ops;
	//port->flags = UPF_BOOT_AUTOCONF;
	bflb_ipc_port.dev = &pdev->dev;


	BL_WR_REG(bflb_ipc_port.membase, IPC_CPU0_IPC_IUSR, 0xFFFFFFFF);	
	bflb_ipc_startup(&bflb_ipc_port);
#if 0
	bflb_ipc->clk = clk_get(port->dev, "baudclk");
	if (IS_ERR(bflb_ipc->clk)) {
		dev_err(&pdev->dev,
			"Unable to get baud rate generator clock\n");
		ret = PTR_ERR(bflb_ipc->clk);
		return ret;
	}
	//clk_prepare_enable(bflb_ipc->clk);
	//clk_rate = clk_get_rate(bflb_ipc->clk);
	// debugfs
	bflb_ipc->debugfs_root = debugfs_create_dir("bluart", NULL);
	bflb_ipc->debug_stats.rx_fifo_overflow_count = 0;
	bflb_ipc->debug_stats.rx_fifo_underflow_count = 0;
	bflb_ipc->debug_stats.rx_fifo_ready_count = 0;
	bflb_ipc->debug_stats.rx_rto_count = 0;
	bflb_ipc->debug_stats.tx_fifo_ready_count = 0;
	bflb_ipc->debug_stats.tx_end_count = 0;
	if (bflb_ipc->debugfs_root) {
		debugfs_create_u32(
			"rx_fifo_overflow_count", 0644, bflb_ipc->debugfs_root,
			&bflb_ipc->debug_stats.rx_fifo_overflow_count);
		debugfs_create_u32(
			"rx_fifo_underflow_count", 0644,
			bflb_ipc->debugfs_root,
			&bflb_ipc->debug_stats.rx_fifo_underflow_count);
		debugfs_create_u32("rx_fifo_ready_count", 0644,
				   bflb_ipc->debugfs_root,
				   &bflb_ipc->debug_stats.rx_fifo_ready_count);
		debugfs_create_u32("rx_rto_count", 0644,
				   bflb_ipc->debugfs_root,
				   &bflb_ipc->debug_stats.rx_rto_count);

		debugfs_create_u32("tx_fifo_ready_count", 0644,
				   bflb_ipc->debugfs_root,
				   &bflb_ipc->debug_stats.tx_fifo_ready_count);
		debugfs_create_u32("tx_end_count", 0644,
				   bflb_ipc->debugfs_root,
				   &bflb_ipc->debug_stats.tx_end_count);
	}

	{
		/* TODO: move to setup */

		//UART_IntMask(UART_DBG_ID, UART_INT_ALL, MASK);
		tmp = readl(port->membase + UART_INT_MASK_OFFSET);
		tmp |= 0xfff;
		writel(tmp, port->membase + UART_INT_MASK_OFFSET);

		//UART_Disable(UART_DBG_ID,UART_TXRX);
		tmp = readl(port->membase + UART_UTX_CONFIG_OFFSET);
		tmp &= UART_CR_UTX_EN_UMSK;
		writel(tmp, port->membase + UART_UTX_CONFIG_OFFSET);
		tmp = readl(port->membase + UART_URX_CONFIG_OFFSET);
		tmp &= UART_CR_URX_EN_UMSK;
		writel(tmp, port->membase + UART_URX_CONFIG_OFFSET);

		//UART_Init(UART_DBG_ID, &uart_dbg_cfg);
		tmp = clk_rate / BFLB_UART_BAUD - 1; /* 2000000bps */
		tmp = (tmp << 16) | tmp;
		writel(tmp, port->membase + UART_BIT_PRD_OFFSET);
		writel(0xf04, port->membase + UART_UTX_CONFIG_OFFSET);
		writel(0x700, port->membase + UART_URX_CONFIG_OFFSET);
		writel(0, port->membase + UART_DATA_CONFIG_OFFSET);
		writel(0, port->membase + UART_SW_MODE_OFFSET);
		//UART_FifoConfig(UART_DBG_ID,&fifoCfg);
		//	writel(0xf0f0080, port->membase + UART_FIFO_CONFIG_1_OFFSET);
		writel(0x00000080, port->membase + UART_FIFO_CONFIG_1_OFFSET);
		writel(0x80, port->membase + UART_FIFO_CONFIG_0_OFFSET);
		//UART_TxFreeRun(UART_DBG_ID,ENABLE);
		writel(0xf04, port->membase + UART_UTX_CONFIG_OFFSET);
		//UART_SetRxTimeoutValue(UART_DBG_ID,80);
		writel(0x4f, port->membase + UART_URX_RTO_TIMER_OFFSET);
		//UART_IntMask(UART_DBG_ID,UART_INT_RX_FIFO_REQ,UNMASK);
		//tmp = readl(port->membase + UART_INT_MASK_OFFSET);
		//tmp &= UART_CR_URX_FIFO_MASK_UMSK;
		//writel(tmp, port->membase + UART_INT_MASK_OFFSET);
		//UART_Enable(UART_DBG_ID,UART_TXRX);
		tmp = readl(port->membase + UART_UTX_CONFIG_OFFSET);
		tmp |= UART_CR_UTX_EN_MSK;
		writel(tmp, port->membase + UART_UTX_CONFIG_OFFSET);
		tmp = readl(port->membase + UART_URX_CONFIG_OFFSET);
		tmp |= UART_CR_URX_EN_MSK;
		writel(tmp, port->membase + UART_URX_CONFIG_OFFSET);
	}

	/* bflb uart does not need to divide the clock by 16 */
	tmp = bflb_ipc_readl(port, UART_BIT_PRD_OFFSET) &
	      UART_CR_UTX_BIT_PRD_MSK;
	port->uartclk = clk_rate * 16 / (tmp + 1);

	uart_add_one_port(&bflb_ipc_driver, port);
#endif
	return 0;
}

static int bflb_ipc_remove(struct platform_device *pdev)
{
	iounmap(bflb_ipc_port.membase);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id bflb_ipc_match[] = {
	{
		.compatible = "bflb-ipc,ipc0",
	},
	{},
};
MODULE_DEVICE_TABLE(of, bflb_ipc_match);
#endif /* CONFIG_OF */

static struct platform_driver bflb_ipc_platform_driver = {
	.probe	= bflb_ipc_probe,
	.remove	= bflb_ipc_remove,
	.driver	= {
		.name		= DRV_NAME,
		.of_match_table	= of_match_ptr(bflb_ipc_match),
	},
};

static int __init bflb_ipc_init(void)
{
	int rc;

	rc = platform_driver_register(&bflb_ipc_platform_driver);
	return rc;
}

static void __exit bflb_ipc_exit(void)
{
	platform_driver_unregister(&bflb_ipc_platform_driver);
}

module_init(bflb_ipc_init);
module_exit(bflb_ipc_exit);

MODULE_DESCRIPTION("BFLB IPC driver");
MODULE_AUTHOR("abc <123@abc.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
