#define CPU_D0
#define IRQ_NUM_BASE 16
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


volatile void *mapped_addr = NULL;  
#define SDH_BASE_ADDR mapped_addr 

#define SDH_CLK_SRC                     (50000000ul)
#define SDH_CLK_INIT                    (400000ul)
#define SDH_CLK_TRANSFER                (50000000ul) 
#if 0
#define SD_CMD_GO_IDLE_STATE                       ((uint8_t)0)
/**
 *  @brief SDH interrupt type definition
 */
#define SDH_INT_CMD_COMPLETED      ((uint32_t)(0x00000001)) /*!< SDH command complete interrupt */
#define SDH_INT_DATA_COMPLETED     ((uint32_t)(0x00000002)) /*!< SDH data complete interrupt */
#define SDH_INT_BLK_GAP_EVENT      ((uint32_t)(0x00000004)) /*!< SDH block gap event interrupt */
#define SDH_INT_DMA_COMPLETED      ((uint32_t)(0x00000008)) /*!< SDH DMA complete interrupt */
#define SDH_INT_BUFFER_WRITE_READY ((uint32_t)(0x00000010)) /*!< SDH buffer write ready interrupt */
#define SDH_INT_BUFFER_READ_READY  ((uint32_t)(0x00000020)) /*!< SDH buffer read ready interrupt */
#define SDH_INT_CARD_INSERT        ((uint32_t)(0x00000040)) /*!< SDH card insert interrupt */
#define SDH_INT_CARD_REMOVE        ((uint32_t)(0x00000080)) /*!< SDH card remove interrupt */
#define SDH_INT_CARD               ((uint32_t)(0x00000100)) /*!< SDH card produce interrupt */
#define SDH_INT_DRIVER_TA          ((uint32_t)(0x00000200)) /*!< SDH driver type A interrupt */
#define SDH_INT_DRIVER_TB          ((uint32_t)(0x00000400)) /*!< SDH driver type B interrupt */
#define SDH_INT_DRIVER_TC          ((uint32_t)(0x00000800)) /*!< SDH driver type C interrupt */
#define SDH_INT_RETUNE_EVENT       ((uint32_t)(0x00001000)) /*!< SDH re-tuning event interrupt */
#define SDH_INT_CMD_TIMEOUT        ((uint32_t)(0x00010000)) /*!< SDH command timeout interrupt */
#define SDH_INT_DATA_CRC_ERROR     ((uint32_t)(0x00200000)) /*!< SDH data CRC error interrupt */
#define SDH_INT_DATA_ENDBIT_ERROR  ((uint32_t)(0x00400000)) /*!< SDH data endbit error interrupt */
#define SDH_INT_CURRENT_ERROR      ((uint32_t)(0x00800000)) /*!< SDH current error interrupt */
#define SDH_INT_AUTO_CMD12_ERROR   ((uint32_t)(0x01000000)) /*!< SDH auto command12 error */
#define SDH_INT_DMA_ERROR          ((uint32_t)(0x02000000)) /*!< SDH DMA error interrupt */
#define SDH_INT_TUNE_ERROR         ((uint32_t)(0x04000000)) /*!< SDH tuning maybe fail,this bit is set when an unrecoverable error is detected in a tuning circuit except during tuning procedure */
#define SDH_INT_SPI_ERROR          ((uint32_t)(0x10000000)) /*!< SDH SPI mode error,read  <SPI Error Token> field in the SPI mode register */
#define SDH_INT_AXI_RESP_ERROR     ((uint32_t)(0x20000000)) /*!< SDH AXI bus response error */
#define SDH_INT_CPL_TIMEOUT_ERROR  ((uint32_t)(0x40000000)) /*!< SDH command completion signal timeout error,this field is applicable for CE-ATA mode only */
#define SDH_INT_CRC_STAT_ERROR     ((uint32_t)(0x80000000)) /*!< SDH CRC status start bit or CRC status end bit or boot ack status, returned from the card in write transaction has errors */
#define SDH_INT_CMD_ERRORS         ((uint32_t)(0x000f0000)) /*!< SDH all command errors */
#define SDH_INT_DATA_ERRORS        ((uint32_t)(0x01700000)) /*!< SDH all data errors */
#define SDH_INT_ALL                ((uint32_t)(0xffffffff)) /*!< SDH all interrupt */

/**
 *  @brief SDH present flag type definition
 */
typedef enum {
    SDH_PRESENT_FLAG_CMD_INHBIT = 0x00000001,       /*!< Command inhbit */
    SDH_PRESENT_FLAG_DATA_INHBIT = 0x00000002,      /*!< Data inhbit */
    SDH_PRESENT_FLAG_DATA_LINE_ACTIVE = 0x00000004, /*!< Data line active */
    SDH_PRESENT_FLAG_RETUNE_REQ = 0x00000008,       /*!< Retuning request */
    SDH_PRESENT_FLAG_TX_ACTIVE = 0x00000100,        /*!< Write trnasfer atcive */
    SDH_PRESENT_FLAG_RX_ACTIVE = 0x00000200,        /*!< Read transfer active */
    SDH_PRESENT_FLAG_BUF_WRITE_ENABLE = 0x00000400, /*!< Buffer write enabled */
    SDH_PRESENT_FLAG_BUF_READ_ENABLE = 0x00000800,  /*!< Buffer read enabled */
    SDH_PRESENT_FLAG_CARD_INSERT = 0x00010000,      /*!< Card intert */
    SDH_PRESENT_FLAG_CARD_STABLE = 0x00020000,      /*!< Card stable */
    SDH_PRESENT_FLAG_CARD_DET = 0x00040000,         /*!< Card detect */
    SDH_PRESENT_FLAG_CARD_WP = 0x00080000,          /*!< Card write protect */
    SDH_PRESENT_FLAG_DATA0_LINE_LEVEL = 0x00100000, /*!< Data 0 line signal level */
    SDH_PRESENT_FLAG_DATA1_LINE_LEVEL = 0x00200000, /*!< Data 1 line signal level */
    SDH_PRESENT_FLAG_DATA2_LINE_LEVEL = 0x00400000, /*!< Data 2 line signal level */
    SDH_PRESENT_FLAG_DATA3_LINE_LEVEL = 0x00800000, /*!< Data 3 line signal level */
    SDH_PRESENT_FLAG_CMD_LINE_LEVEL = 0x01000000,   /*!< Command line signal level */
} SDH_Present_Flag_Type;



/**
 *  @brief SDH status type definition
 */
typedef enum {
    SDH_STAT_SUCCESS = 0,        /*!< SDH status success */
    SDH_STAT_BUSY,               /*!< SDH status busy */
    SDH_STAT_INVLAID_PARA,       /*!< SDH status invalid parameter */
    SDH_STAT_NOT_SUPPORT,        /*!< SDH not support this feature */
    SDH_STAT_OUTOF_RANGE,        /*!< SDH out of range */
    SDH_STAT_PREPARE_DESC,       /*!< SDH status prepare ADMA descriptor */
    SDH_STAT_SEND_CMD_FAIL,      /*!< SDH status send command fail */
    SDH_STAT_SEND_DATA_FAIL,     /*!< SDH status send data fail */
    SDH_STAT_RECV_DATA_FAIL,     /*!< SDH status receive data fail */
    SDH_STAT_DMA_ADDR_NOT_ALIGN, /*!< SDH status DMA address not align */
    SDH_STAT_RETUNE_REQ,         /*!< SDH status re-tune request */
    SDH_STAT_TUNE_ERROR,         /*!< SDH status tune error */
} SDH_Stat_Type;

/**
 *  @brief SDH Command type definition
 */
typedef enum {
    SDH_CMD_NORMAL,  /*!< SDH command type:normal */
    SDH_CMD_SUSPEND, /*!< SDH command type:suspend */
    SDH_CMD_RESUME,  /*!< SDH command type:resume */
    SDH_CMD_ABORT,   /*!< SDH command type:abort */
    SDH_CMD_EMPTY,   /*!< SDH command type:empty */
} SDH_Cmd_Type;
/**
 *  @brief SDH response type definition
 */
typedef enum {
    SDH_RESP_NONE, /*!< SDH response type:none */
    SDH_RESP_136LEN,
    SDH_RESP_48LEN,
    SDH_RESP_48LEN_BUSY,
    SDH_RESP_R1,  /*!< SDH response type:r1 */
    SDH_RESP_R1B, /*!< SDH response type:r1b */
    SDH_RESP_R2,  /*!< SDH response type:r2 */
    SDH_RESP_R3,  /*!< SDH response type:r3 */
    SDH_RESP_R4,  /*!< SDH response type:r4 */
    SDH_RESP_R5,  /*!< SDH response type:r5 */
    SDH_RESP_R5B, /*!< SDH response type:r5b */
    SDH_RESP_R6,  /*!< SDH response type:r6 */
    SDH_RESP_R7,  /*!< SDH response type:r7 */
} SDH_Resp_Type;

/**
 *  @brief SDH trnasfer flag type definition
 */
typedef enum {
    SDH_TRANS_FLAG_NONE = 0x00000000,
    SDH_TRANS_FLAG_EN_DMA = 0x00000001,                /*!< Enable DMA */
    SDH_TRANS_FLAG_EN_BLK_COUNT = 0x00000002,          /*!< Enable block count */
    SDH_TRANS_FLAG_EN_AUTO_CMD12 = 0x00000004,         /*!< Enable auto CMD12 */
    SDH_TRANS_FLAG_EN_AUTO_CMD23 = 0x00000008,         /*!< Enable auto CMD23 */
    SDH_TRANS_FLAG_READ_DATA = 0x00000010,             /*!< Enable read data */
    SDH_TRANS_FLAG_MULTI_BLK = 0x00000020,             /*!< Enable multi-block data operation */
    SDH_TRANS_FLAG_RESP_136BITS = 0x00010000,          /*!< Response is 136 bits length */
    SDH_TRANS_FLAG_RESP_48BITS = 0x00020000,           /*!< Response is 48 bits length */
    SDH_TRANS_FLAG_RESP_48BITS_WITH_BUSY = 0x00030000, /*!< Response is 48 bits length with busy status */
    SDH_TRANS_FLAG_EN_CRC_CHECK = 0x00080000,          /*!< Enable CRC check */
    SDH_TRANS_FLAG_EN_INDEX_CHECK = 0x00100000,        /*!< Enable index check */
    SDH_TRANS_FLAG_DATA_PRESENT = 0x00200000,          /*!< Data present */
    SDH_TRANS_FLAG_SUSPEND = 0x00400000,               /*!< Suspend command */
    SDH_TRANS_FLAG_RESUME = 0x00800000,                /*!< Resume command */
    SDH_TRANS_FLAG_ABORT = 0x00C00000,                 /*!< Abort command */
} SDH_Trans_Flag_Type;



#define SDH_EnableIntSource(mask)                                                 \
    do {                                                                          \
        BL_WR_REG(SDH_BASE_ADDR, SDH_SD_NORMAL_INT_STATUS_INT_EN,                      \
                  BL_RD_REG(SDH_BASE_ADDR, SDH_SD_NORMAL_INT_STATUS_INT_EN) | (mask)); \
    } while (0)
#define SDH_DisableIntSource(mask)                                                   \
    do {                                                                             \
        BL_WR_REG(SDH_BASE_ADDR, SDH_SD_NORMAL_INT_STATUS_INT_EN,                         \
                  BL_RD_REG(SDH_BASE_ADDR, SDH_SD_NORMAL_INT_STATUS_INT_EN) & (~(mask))); \
    } while (0)
#define SDH_GetIntStatus()       BL_RD_REG(SDH_BASE_ADDR, SDH_SD_NORMAL_INT_STATUS);
#define SDH_GetIntEnableStatus() BL_RD_REG(SDH_BASE_ADDR, SDH_SD_NORMAL_INT_STATUS_INT_EN);
#define SDH_ClearIntStatus(mask) BL_WR_REG(SDH_BASE_ADDR, SDH_SD_NORMAL_INT_STATUS, (mask));

/**
 *  @brief SDH command config structure type definition
 */
typedef struct
{
    uint32_t index;         /*!< SDH command index */
    uint32_t argument;      /*!< SDH command argument */
    SDH_Cmd_Type type;      /*!< SDH command type */
    SDH_Resp_Type respType; /*!< SDH command response type */
    uint32_t response[4U];  /*!< SDH response for this command */
    uint32_t flag;          /*!< SDH cmd flag */
} SDH_CMD_Cfg_Type;


#endif


volatile void *xram_addr = NULL;
volatile void *ipc_reg = NULL;
volatile void *ipc2_reg = NULL;
volatile int ipi_cnt = 0;
static dev_t first_dev;  
static struct cdev my_cdev;  
static struct class *my_class; 

#define BL_WR_WORD(addr, val)  ((*(volatile uint32_t *)(uintptr_t)(addr)) = (val))
#define BL_WR_REG(addr, regname, val)             BL_WR_WORD(addr + regname##_OFFSET, val)

#define BL_RD_WORD(addr)       (*((volatile uint32_t *)(uintptr_t)(addr)))
#define BL_RD_REG(addr, regname)                  BL_RD_WORD(addr + regname##_OFFSET)

#define BL_SET_REG_BITS_VAL(val, bitname, bitval) (((val)&bitname##_UMSK) | ((uint32_t)(bitval) << bitname##_POS))

#define BL_RD_SHORT(addr)      (*((volatile uint16_t *)(uintptr_t)(addr)))
#define BL_WR_SHORT(addr, val) ((*(volatile uint16_t *)(uintptr_t)(addr)) = (val))
#define BL_WR_REG16(addr, regname, val)           BL_WR_SHORT(addr + regname##_OFFSET, val)

void read_sd_block(unsigned long index);
void write_sd_block(unsigned long index);

/****************************************************************************/ /**
 * @brief  SDH enable interrupt
 *
 * @param  cmd: Transfer data config pointer
 *
 * @return SDH status
 *
*******************************************************************************/
SDH_Stat_Type SDH_GetCmdResp(SDH_CMD_Cfg_Type *cmd)
{
    if (cmd->respType != SDH_RESP_NONE) {
        cmd->response[0U] = BL_RD_REG(SDH_BASE_ADDR, SDH_SD_RESP_0);
        cmd->response[1U] = BL_RD_REG(SDH_BASE_ADDR, SDH_SD_RESP_2);
        cmd->response[2U] = BL_RD_REG(SDH_BASE_ADDR, SDH_SD_RESP_4);
        cmd->response[3U] = BL_RD_REG(SDH_BASE_ADDR, SDH_SD_RESP_6);
    }

    return SDH_STAT_SUCCESS;
}



/****************************************************************************
 * @brief  SDH init function
 *
 * @param  cfg: SDH config pointer
 *
 * @return None
 *
*******************************************************************************/
void SDH_Ctrl_Init(const SDH_Cfg_Type *cfg)
{
    uint32_t tmpVal;

    //SDH_SetSdClock(cfg->srcClock, cfg->busClock);
    SDH_DisableDMA();

    /* Set SDH config */
    tmpVal = BL_RD_REG16(SDH_BASE_ADDR, SDH_SD_HOST_CTRL);

    if (cfg->dataWidth == SDH_DATA_BUS_WIDTH_8BITS) {
        tmpVal = BL_SET_REG_BIT(tmpVal, SDH_EX_DATA_WIDTH);
    } else {
        tmpVal = BL_CLR_REG_BIT(tmpVal, SDH_EX_DATA_WIDTH);
        tmpVal = BL_SET_REG_BITS_VAL(tmpVal, SDH_DATA_WIDTH, cfg->dataWidth);
    }

    tmpVal = BL_SET_REG_BITS_VAL(tmpVal, SDH_HI_SPEED_EN, cfg->highSpeed);
    tmpVal = BL_SET_REG_BITS_VAL(tmpVal, SDH_SD_BUS_VLT, cfg->volt);

    BL_WR_REG16(SDH_BASE_ADDR, SDH_SD_HOST_CTRL, tmpVal);

    if (cfg->vlot18Enable) {
        tmpVal = BL_RD_REG16(SDH_BASE_ADDR, SDH_HOST_CTRL_2);
        tmpVal = BL_SET_REG_BITS_VAL(tmpVal, SDH_UHS_MODE_SEL, cfg->speed);
        tmpVal = BL_SET_REG_BITS_VAL(tmpVal, SDH_SDH_V18_EN, cfg->vlot18Enable);
        BL_WR_REG16(SDH_BASE_ADDR, SDH_HOST_CTRL_2, tmpVal);
    }

    tmpVal = BL_RD_REG(SDH_BASE_ADDR, SDH_TX_CFG_REG);
    tmpVal = BL_SET_REG_BIT(tmpVal, SDH_TX_INT_CLK_SEL);
    BL_WR_REG(SDH_BASE_ADDR, SDH_TX_CFG_REG, tmpVal);

    //SDH_EnableStatus();

#ifndef BFLB_USE_HAL_DRIVER
    //Interrupt_Handler_Register(SDH_IRQn, SDH_MMC1_IRQHandler);
#endif
}


static int my_driver_open(struct inode *inode, struct file *file)  
{  
    printk("my_driver_open, ipi_cnt:%d\n", ipi_cnt);
    if(ipc_reg)
    {
	    //BL_WR_REG(ipc_reg, IPC_CPU1_IPC_ISWR, 1 << 1);
    }
    return 0;  
}  
  
static int my_driver_release(struct inode *inode, struct file *file)  
{  
    return 0;  
}  
  
static ssize_t my_driver_read(struct file *filp, char *buffer, size_t length, loff_t * offset)  
{  
    size_t copy_size;
    if (*offset >= SIZE) {  
        return 0;  // 已到达文件末尾，没有更多数据可读  
    }  
    if (*offset + length > SIZE) {  
        length = SIZE - *offset;  // 防止读取超过文件大小  
    }  

    //asm volatile("fence");

    copy_size = copy_to_user(buffer, (const void *)(mapped_addr + *offset), length);

    (void)copy_size;
    //flush_icache_pte(mapped_addr/4096);

    *offset += length;  // 更新偏移量以供下一次读取使用  
    return length;  // 返回实际读取的字节数  
}  


uint32_t str_to_hex(const char *str)
{
	        uint32_t ret = 0;

		        if((str[0]=='0') && ((str[1] == 'x') || (str[1] == 'X')))
				        {
						                if(strlen(str) > 10)
									                        return ret;
								                else
											                        str += 2;
										        }
			        else if(strlen(str) > 8)
					        {
							                return ret;
									        }


				        sscanf(str, "%08x", &ret); //
					        return ret;
}

uint32_t str_to_int(const char *str)
{
	uint32_t ret = 0;

	sscanf(str, "%d", &ret); //
	return ret;
}


static ssize_t my_driver_write(struct file *filp, const char *buffer, size_t length, loff_t * offset)  
{  
    char buf[128] = {0};
    size_t copy_size;
    if (*offset >= SIZE) {  
        return -EINVAL;  // 已到达文件末尾，无法写入更多数据  
    }  
    if (*offset + length > SIZE) {  
        length = SIZE - *offset;  // 防止写入超过文件大小  
    }  

#if 1
    copy_size = copy_from_user((void *)buf, buffer, length);
    (void)copy_size;
    {
	uint32_t index = str_to_int(buf);
    	if(index > 10000)
    	{
	    index -= 10000;
	    write_sd_block(index);
    	}
   	read_sd_block(index);
    }
   
    //memcpy_toio(xram_addr, buf, length);
    //asm volatile("l2cache.ciall");
    //csi_dcache_invalid_range(xram_addr, 4);
#else    
    uint8_t buf[128] = {0};
    copy_from_user(buf, buffer, length);
    writel(*((uint32_t *)buf), mapped_addr);
    //asm volatile("fence");
#endif

    *offset += length;  // 更新偏移量以供下一次写入使用  
    return length;  // 返回实际写入的字节数  
}  
  
static const struct file_operations my_driver_fops = {  
    .owner = THIS_MODULE,  
    .read = my_driver_read,  
    .write = my_driver_write,  
    .open = my_driver_open,  
    .release = my_driver_release,  
};  
  

irqreturn_t ipi_interrupt_handler(int irq, void *dev_id)  
{  
	//handle
	ipi_cnt++;   
	return IRQ_HANDLED;  
}  


/****************************************************************************/ /**
 * @brief  SDH send command
 *
 * @param  cmd: command config pointer
 *
 * @return None
 *
*******************************************************************************/
void SDH_SendCommand(SDH_CMD_Cfg_Type *cmd)
{
    uint32_t tmpVal;
    uint32_t flag = cmd->flag;
    //uint32_t flag = 0;

    //printk("SDH_BASE_ADDR:0x%px\n", SDH_BASE_ADDR);
    tmpVal = BL_RD_REG(SDH_BASE_ADDR, SDH_SD_PRESENT_STATE_1);

    //printk("tmpVal:%x\r\n", tmpVal);
    if (((tmpVal & SDH_PRESENT_FLAG_CMD_INHBIT) == 0U) && (cmd->type != SDH_CMD_EMPTY)) {
        /* Define the flag corresponding to each response type. */
        switch (cmd->respType) {
            case SDH_RESP_NONE:
                break;

            case SDH_RESP_R1: /* Response 1 */
            case SDH_RESP_R5: /* Response 5 */
            case SDH_RESP_R6: /* Response 6 */
            case SDH_RESP_R7: /* Response 7 */
                flag |= (SDH_TRANS_FLAG_RESP_48BITS | SDH_TRANS_FLAG_EN_CRC_CHECK | SDH_TRANS_FLAG_EN_INDEX_CHECK);
                break;

            case SDH_RESP_R1B: /* Response 1 with busy */
            case SDH_RESP_R5B: /* Response 5 with busy */
                flag |= (SDH_TRANS_FLAG_RESP_48BITS_WITH_BUSY | SDH_TRANS_FLAG_EN_CRC_CHECK | SDH_TRANS_FLAG_EN_INDEX_CHECK);
                break;

            case SDH_RESP_R2: /* Response 2 */
                flag |= (SDH_TRANS_FLAG_RESP_136BITS | SDH_TRANS_FLAG_EN_CRC_CHECK);
                break;

            case SDH_RESP_R3: /* Response 3 */
            case SDH_RESP_R4: /* Response 4 */
                flag |= (SDH_TRANS_FLAG_RESP_48BITS);
                break;

            default:
                break;
        }

        /* Config command index */
        tmpVal = flag >> 16;
        tmpVal = BL_SET_REG_BITS_VAL(tmpVal, SDH_CMD_TYPE, cmd->type);
        tmpVal = BL_SET_REG_BITS_VAL(tmpVal, SDH_CMD_INDEX, cmd->index);

        /* Config command argument */
        BL_WR_REG(SDH_BASE_ADDR, SDH_SD_ARG_LOW, cmd->argument);

        /* Config command index */
        BL_WR_REG16(SDH_BASE_ADDR, SDH_SD_CMD, tmpVal);
	//printf("SDH_SD_CMD\r\n");
    }

    /* FIXME:how to enable again */
    if (cmd->type == SDH_CMD_EMPTY) {
        SDH_DisableIntSource(SDH_INT_CMD_COMPLETED);
    }
}
/****************************************************************************/ /**
 * @brief  SDH wait command send done
 *
 * @param  cmd: Command config pointer
 *
 * @return Command deal status
 *
*******************************************************************************/
SDH_Stat_Type SDH_WaitCommandDone(SDH_CMD_Cfg_Type *cmd)
{
    uint32_t intStatus = 0U;
    uint32_t count = 0;
    SDH_Stat_Type stat = SDH_STAT_SUCCESS;

    /* Wait command complete or encounters error. */
    while (!(intStatus & (SDH_INT_CMD_COMPLETED | SDH_INT_CMD_ERRORS | SDH_INT_TUNE_ERROR))) {
        intStatus = SDH_GetIntStatus();
	count++;
	if(count%2000==0)
		printk("intStatus:%x\r\n", intStatus);
    }

    if ((intStatus & SDH_INT_TUNE_ERROR) != 0U) {
        stat = SDH_STAT_TUNE_ERROR;
    } else if ((intStatus & SDH_INT_CMD_ERRORS) != 0U) {
        stat = SDH_STAT_SEND_CMD_FAIL;
    }

    /* Receive response when command completes successfully. */
    if (stat == SDH_STAT_SUCCESS) {
        stat = SDH_GetCmdResp(cmd);
    }

    SDH_ClearIntStatus(SDH_INT_CMD_COMPLETED | SDH_INT_CMD_ERRORS | SDH_INT_TUNE_ERROR);

    return stat;
}

static SD_Error SDH_WaitStatus   =  SD_WAITING;
static SDH_CMD_Cfg_Type             SDH_CMD_Cfg_TypeInstance;
static SDH_Data_Cfg_Type            SDH_Data_Cfg_TypeInstance;
static SDH_Trans_Cfg_Type           SDH_Trans_Cfg_TypeInstance={&SDH_Data_Cfg_TypeInstance,&SDH_CMD_Cfg_TypeInstance};
static SDH_DMA_Cfg_Type   SDH_DMA_Cfg_TypeInstance;
static SDH_ADMA2_Desc_Type adma2Entries[16];

/* Private variables ---------------------------------------------------------*/
static uint32_t CardType =  SDIO_STD_CAPACITY_SD_CARD_V2_0;

sd_card_t gSDCardInfo;
static sd_card_t *pSDCardInfo = &gSDCardInfo;

/****************************************************************************/ /**
 * @brief  SDH enable interrupt
 *
 * @param  None
 *
 * @return None
 *
*******************************************************************************/
void SDH_ITConfig(uint32_t SDH_IT_FLAG, BL_Fun_Type NewState)
{
    if (NewState != DISABLE) {
        /* Enable the SDH interrupts */
        SDH_EnableIntSource(SDH_IT_FLAG);
    } else {
        /* Disable the SDH interrupts */
        SDH_DisableIntSource(SDH_IT_FLAG);
    }
}


SD_Error SDH_PowerON(void)
{
	SD_Error errorstatus = SD_OK;
	SDH_Stat_Type stat = SDH_STAT_SUCCESS;
	uint32_t SDType = SD_HIGH_CAPACITY;
	uint32_t response = 0, count = 0, validvoltage = 0;
	
	
	/*CMD0: GO_IDLE_STATE, send card to reset state*/
	SDH_CMD_Cfg_TypeInstance.index = SD_CMD_GO_IDLE_STATE;
	SDH_CMD_Cfg_TypeInstance.argument = 0;
	SDH_CMD_Cfg_TypeInstance.type = SDH_CMD_NORMAL;
	SDH_CMD_Cfg_TypeInstance.respType = SDH_RESP_NONE;
	SDH_CMD_Cfg_TypeInstance.flag = SDH_TRANS_FLAG_NONE;

	SDH_ClearIntStatus(SDH_INT_CMD_COMPLETED | SDH_INT_CMD_ERRORS);	

	SDH_SendCommand(&SDH_CMD_Cfg_TypeInstance);	
	//printf("SDH_SendCommand\r\n");
	stat = SDH_WaitCommandDone(&SDH_CMD_Cfg_TypeInstance);
	printk("stat:%d\r\n", stat);
	if(stat != SDH_STAT_SUCCESS)
	{
		return SD_CMD_ERROR;
	}

	/* CMD8: SEND_IF_COND */
	SDH_CMD_Cfg_TypeInstance.index = SD_CMD_HS_SEND_EXT_CSD;
	SDH_CMD_Cfg_TypeInstance.argument = SD_CHECK_PATTERN;
	SDH_CMD_Cfg_TypeInstance.type = SDH_CMD_NORMAL;
	SDH_CMD_Cfg_TypeInstance.respType = SDH_RESP_R7;
	SDH_CMD_Cfg_TypeInstance.flag = SDH_TRANS_FLAG_NONE;
	
	SDH_SendCommand(&SDH_CMD_Cfg_TypeInstance);	
	stat = SDH_WaitCommandDone(&SDH_CMD_Cfg_TypeInstance);
	if(stat != SDH_STAT_SUCCESS)
	{
		return SD_CMD_ERROR;
	}
	
	
	//printf("R7 is: 0x%02x.\r\n",SDH_CMD_Cfg_TypeInstance.response[0]);
	
	/* SD Card 2.0 */
  	CardType = SDIO_STD_CAPACITY_SD_CARD_V2_0;	
	/* parameter for ACMD41, to query card is sdsc or sdhc? */
  	SDType = SD_HIGH_CAPACITY;		
	
	/* CMD55: SEND_IF_COND */
	SDH_CMD_Cfg_TypeInstance.index = SD_CMD_APP_CMD;
	SDH_CMD_Cfg_TypeInstance.argument = 0;
	SDH_CMD_Cfg_TypeInstance.type = SDH_CMD_NORMAL;
	SDH_CMD_Cfg_TypeInstance.respType = SDH_RESP_R1;
	SDH_CMD_Cfg_TypeInstance.flag = SDH_TRANS_FLAG_NONE;
	
	SDH_SendCommand(&SDH_CMD_Cfg_TypeInstance);	
	stat = SDH_WaitCommandDone(&SDH_CMD_Cfg_TypeInstance);
	if(stat != SDH_STAT_SUCCESS){
		return SD_CMD_ERROR;
	}else if(SDH_CMD_Cfg_TypeInstance.response[0] & SD_CSR_ERRORBITS){
		return SD_CMD_RESP1_ERROR;
	}
	do{
		/*loop send ACMD41*/
		SDH_CMD_Cfg_TypeInstance.index = SD_CMD_APP_CMD;
		SDH_CMD_Cfg_TypeInstance.argument = 0;
		SDH_CMD_Cfg_TypeInstance.type = SDH_CMD_NORMAL;
		SDH_CMD_Cfg_TypeInstance.respType = SDH_RESP_R1;
		SDH_CMD_Cfg_TypeInstance.flag = SDH_TRANS_FLAG_NONE;
		
		SDH_SendCommand(&SDH_CMD_Cfg_TypeInstance);	
		stat = SDH_WaitCommandDone(&SDH_CMD_Cfg_TypeInstance);
		if(stat != SDH_STAT_SUCCESS){
			return SD_CMD_ERROR;
		}else if(SDH_CMD_Cfg_TypeInstance.response[0] & SD_CSR_ERRORBITS){
			return SD_CMD_RESP1_ERROR;
		}
		/*ACMD41*/
		SDH_CMD_Cfg_TypeInstance.index = SD_CMD_SD_APP_OP_COND;
		SDH_CMD_Cfg_TypeInstance.argument = SD_VOLTAGE_WINDOW_SD | SDType;
		SDH_CMD_Cfg_TypeInstance.type = SDH_CMD_NORMAL;
		SDH_CMD_Cfg_TypeInstance.respType = SDH_RESP_R3;
		SDH_CMD_Cfg_TypeInstance.flag = SDH_TRANS_FLAG_NONE;	
		SDH_SendCommand(&SDH_CMD_Cfg_TypeInstance);	
		stat = SDH_WaitCommandDone(&SDH_CMD_Cfg_TypeInstance);
		if(stat != SDH_STAT_SUCCESS){
			return SD_CMD_ERROR;
		}
		response = SDH_CMD_Cfg_TypeInstance.response[0];
		validvoltage = (((response >> 31) == 1) ? 1 : 0);	
		count++;	
	}while(  (!validvoltage) && (count < SD_MAX_VOLT_TRIAL)  );

	pSDCardInfo->ocr = response;
	if (response &= SD_HIGH_CAPACITY) {      
		/* change from sdsc to sdhc */
		CardType = SDIO_HIGH_CAPACITY_SD_CARD; 
	}		
	
	return errorstatus;
}

static void SD_DecodeCid(sd_card_t *card, uint32_t *rawCid)
{
    sd_cid_t *cid;

    cid = &(card->cid);
    cid->manufacturerID = (uint8_t)((rawCid[3U] & 0xFF0000U) >> 16U);
    cid->applicationID = (uint16_t)((rawCid[3U] & 0xFFFFU) >> 0U);

    cid->productName[0U] = (uint8_t)((rawCid[1U] & 0xFF000000U) >> 24);
    cid->productName[1U] = (uint8_t)((rawCid[2U] & 0xFF) >> 0U);
    cid->productName[2U] = (uint8_t)((rawCid[2U] & 0xFF00U) >> 8U);
    cid->productName[3U] = (uint8_t)((rawCid[2U] & 0xFF0000U) >> 16U);
    cid->productName[4U] = (uint8_t)((rawCid[2U] & 0xFF000000U) >> 24U);

    cid->productVersion = (uint8_t)((rawCid[1U] & 0xFF0000U) >> 16U);

    cid->productSerialNumber = (uint32_t)((rawCid[1U] & 0xFFFFU) << 16U);
    cid->productSerialNumber |= (uint32_t)((rawCid[0U] & 0xFFFF0000U) >> 16U);

    cid->manufacturerData = (uint16_t)((rawCid[0U] & 0xFFFU) >> 0U);
}
static void SD_DecodeCsd(sd_card_t *card, uint32_t *rawCsd)
{
    sd_csd_t *csd;

    csd = &(card->csd);
    csd->csdStructure = (uint8_t)((rawCsd[3U] & 0xC00000U) >> 22U);
    csd->dataReadAccessTime1 = (uint8_t)((rawCsd[3U] & 0xFF00U) >> 8U);
    csd->dataReadAccessTime2 = (uint8_t)((rawCsd[3U] & 0xFFU) >> 0U);
	
    csd->transferSpeed = (uint8_t)((rawCsd[2U] & 0xFF000000U) >> 24);
	  csd->cardCommandClass = (uint16_t)((rawCsd[2U] & 0xFFF000U) >> 12U);
    csd->readBlockLength = (uint8_t)((rawCsd[2U] & 0xF00U) >> 8U);

		csd->deviceSize = (uint32_t)((rawCsd[1U] & 0x3FFFFF00U) >> 8U);
		card->blockSize = 512;
		card->blockCount = ((csd->deviceSize + 1U) * 1024U);
}

SD_Error SDH_InitializeCards(void)
{
  SD_Error errorstatus = SD_OK;
	SDH_Stat_Type stat = SDH_STAT_SUCCESS;
 	
	/* CMD2: SD_CMD_ALL_SEND_CID */
	SDH_CMD_Cfg_TypeInstance.index = SD_CMD_ALL_SEND_CID;
	SDH_CMD_Cfg_TypeInstance.argument = 0;
	SDH_CMD_Cfg_TypeInstance.type = SDH_CMD_NORMAL;
	SDH_CMD_Cfg_TypeInstance.respType = SDH_RESP_R2;
	SDH_CMD_Cfg_TypeInstance.flag = SDH_TRANS_FLAG_NONE;
	
	SDH_SendCommand(&SDH_CMD_Cfg_TypeInstance);	
	stat = SDH_WaitCommandDone(&SDH_CMD_Cfg_TypeInstance);
	if(stat != SDH_STAT_SUCCESS){
		return SD_CMD_ERROR;
	}
//	CID_Tab[0] = SDH_CMD_Cfg_TypeInstance.response[0];
//	CID_Tab[1] = SDH_CMD_Cfg_TypeInstance.response[1];
//	CID_Tab[2] = SDH_CMD_Cfg_TypeInstance.response[2];
//	CID_Tab[3] = SDH_CMD_Cfg_TypeInstance.response[3];

//	SDH_MSG("CID is: 0x%02x-0x%02x-0x%02x-0x%02x.\r\n",
//				SDH_CMD_Cfg_TypeInstance.response[0],SDH_CMD_Cfg_TypeInstance.response[1],SDH_CMD_Cfg_TypeInstance.response[2],SDH_CMD_Cfg_TypeInstance.response[3]);

	pSDCardInfo->rawCid[0] = SDH_CMD_Cfg_TypeInstance.response[0];
	pSDCardInfo->rawCid[1] = SDH_CMD_Cfg_TypeInstance.response[1];
	pSDCardInfo->rawCid[2] = SDH_CMD_Cfg_TypeInstance.response[2];
	pSDCardInfo->rawCid[3] = SDH_CMD_Cfg_TypeInstance.response[3];
	
	SD_DecodeCid(pSDCardInfo,pSDCardInfo->rawCid);
	
	/*CMD3: send relative card address*/
	SDH_CMD_Cfg_TypeInstance.index = SD_CMD_SET_REL_ADDR;
	SDH_CMD_Cfg_TypeInstance.argument = 0;
	SDH_CMD_Cfg_TypeInstance.type = SDH_CMD_NORMAL;
	SDH_CMD_Cfg_TypeInstance.respType = SDH_RESP_R6;
	SDH_CMD_Cfg_TypeInstance.flag = SDH_TRANS_FLAG_NONE;
	
	SDH_SendCommand(&SDH_CMD_Cfg_TypeInstance);	
	stat = SDH_WaitCommandDone(&SDH_CMD_Cfg_TypeInstance);
	if(stat != SDH_STAT_SUCCESS){
		return SD_CMD_ERROR;
	}
	//SDH_MSG("RCA is: 0x%02x. status is 0x%02x. \r\n",SDH_CMD_Cfg_TypeInstance.response[0]>>16, SDH_CMD_Cfg_TypeInstance.response[0]&0xffff);

//	RCA = SDH_CMD_Cfg_TypeInstance.response[0]>>16;   /*global variable to save rca*/
	pSDCardInfo->relativeAddress = SDH_CMD_Cfg_TypeInstance.response[0]>>16;
	
	/*CMD9: send card-specific data(CSD)*/
	SDH_CMD_Cfg_TypeInstance.index = SD_CMD_SEND_CSD;
	SDH_CMD_Cfg_TypeInstance.argument = (uint32_t)( (pSDCardInfo->relativeAddress) << 16);
	SDH_CMD_Cfg_TypeInstance.type = SDH_CMD_NORMAL;
	SDH_CMD_Cfg_TypeInstance.respType = SDH_RESP_R2;
	SDH_CMD_Cfg_TypeInstance.flag = SDH_TRANS_FLAG_NONE;
	
	SDH_SendCommand(&SDH_CMD_Cfg_TypeInstance);	
	stat = SDH_WaitCommandDone(&SDH_CMD_Cfg_TypeInstance);
	if(stat != SDH_STAT_SUCCESS){
		return SD_CMD_ERROR;
	}	
//	CSD_Tab[0] = SDH_CMD_Cfg_TypeInstance.response[0];
//	CSD_Tab[1] = SDH_CMD_Cfg_TypeInstance.response[1];
//	CSD_Tab[2] = SDH_CMD_Cfg_TypeInstance.response[2];
//	CSD_Tab[3] = SDH_CMD_Cfg_TypeInstance.response[3];
//	SDH_MSG("CSD is: 0x%02x-0x%02x-0x%02x-0x%02x.\r\n",
//						SDH_CMD_Cfg_TypeInstance.response[0],SDH_CMD_Cfg_TypeInstance.response[1],SDH_CMD_Cfg_TypeInstance.response[2],SDH_CMD_Cfg_TypeInstance.response[3]);

	pSDCardInfo->rawCsd[0] = SDH_CMD_Cfg_TypeInstance.response[0];
	pSDCardInfo->rawCsd[1] = SDH_CMD_Cfg_TypeInstance.response[1];
	pSDCardInfo->rawCsd[2] = SDH_CMD_Cfg_TypeInstance.response[2];
	pSDCardInfo->rawCsd[3] = SDH_CMD_Cfg_TypeInstance.response[3];	
	
	SD_DecodeCsd(pSDCardInfo,pSDCardInfo->rawCsd);
	return 	errorstatus;
}


static uint32_t bootDummy = 0;


/****************************************************************************/ /**
 * @brief  SDH read data port
 *
 * @param  dataCfg: Transfer data config pointer
 *
 * @return Read length
 *
*******************************************************************************/
uint32_t SDH_ReadDataPort(SDH_Data_Cfg_Type *dataCfg)
{
    uint32_t tmpVal;
    uint32_t totalLen, recvLen = 0;

    totalLen = (dataCfg->blockCount * dataCfg->blockSize);
    tmpVal = BL_RD_REG(SDH_BASE_ADDR, SDH_SD_TRANSFER_MODE);

    //printk("totalLen:%d\n", totalLen);
    if (!BL_IS_REG_BIT_SET(tmpVal, SDH_DMA_EN)) {
    	//printk("ok dataCfg->rxData:%px\n", dataCfg->rxData);
	    do {
            dataCfg->rxData[dataCfg->rxDataLen++] = BL_RD_REG(SDH_BASE_ADDR, SDH_SD_BUFFER_DATA_PORT_0);
            //if(dataCfg->rxData[dataCfg->rxDataLen-1])
	    //	   printk("%d data:%x", dataCfg->rxDataLen-1, dataCfg->rxData[dataCfg->rxDataLen-1]);
	    recvLen += 4;
        } while (recvLen < totalLen);
    }

    return recvLen;
}

/****************************************************************************/ /**
 * @brief  SDH read data port
 *
 * @param  dataCfg: Transfer data config pointer
 *
 * @return Read length
 *
*******************************************************************************/
SDH_Stat_Type SDH_ReadDataPortBlock(SDH_Data_Cfg_Type *dataCfg)
{
    uint32_t intStatus = 0U;
    uint32_t count = 0;
    SDH_Stat_Type error = SDH_STAT_SUCCESS;

    /* Make sure the buffer size is 4 byte aligned */
    if (dataCfg->blockSize % sizeof(uint32_t) != 0U) {
        dataCfg->blockSize += sizeof(uint32_t) - (dataCfg->blockSize % sizeof(uint32_t));
    }

    while (!(intStatus & (SDH_INT_BUFFER_READ_READY | SDH_INT_DATA_ERRORS | SDH_INT_TUNE_ERROR))) {
        intStatus = SDH_GetIntStatus();
	count++;
	if(count%100000==0)
		printk("intStatus:%x", intStatus);
    }

    /* during std tuning process, software do not need to read data, but wait BRR is enough */
    if ((dataCfg->dataType == SDH_TRANS_DATA_TUNING) && (intStatus & SDH_INT_BUFFER_READ_READY)) {
        SDH_ClearIntStatus(SDH_INT_BUFFER_READ_READY);
        return SDH_STAT_SUCCESS;
    } else if ((intStatus & SDH_INT_TUNE_ERROR) != 0U) {
        SDH_ClearIntStatus(SDH_INT_TUNE_ERROR);
        /* if tuning error occur ,return directly */
        error = SDH_STAT_TUNE_ERROR;
    } else if ((intStatus & SDH_INT_DATA_ERRORS) != 0U) {
        if (!(dataCfg->enableIgnoreError)) {
            error = SDH_STAT_RECV_DATA_FAIL;
        }

        /* clear data error flag */
        SDH_ClearIntStatus(SDH_INT_DATA_ERRORS);
    }

    if (error == SDH_STAT_SUCCESS) {
        SDH_ReadDataPort(dataCfg);
        /* clear buffer read ready */
        SDH_ClearIntStatus(SDH_INT_BUFFER_READ_READY);
        intStatus = 0U;
    }

    /* Clear data complete flag after the last read operation. */
    SDH_ClearIntStatus(SDH_INT_DATA_COMPLETED);

    //printk("end\n");
    return error;

}

/****************************************************************************/ /**
 * @brief  SDH write data port
 *
 * @param  dataCfg: Transfer data config pointer
 *
 * @return write length
 *
*******************************************************************************/
uint32_t SDH_WriteDataPort(SDH_Data_Cfg_Type *dataCfg)
{
    uint32_t tmpVal;

    tmpVal = BL_RD_REG(SDH_BASE_ADDR, SDH_SD_TRANSFER_MODE);

    if (!BL_IS_REG_BIT_SET(tmpVal, SDH_DMA_EN)) {
        BL_WR_REG(SDH_BASE_ADDR, SDH_SD_BUFFER_DATA_PORT_0, dataCfg->txData[dataCfg->txDataLen++]);
    }

    return 4;
}

/****************************************************************************/ /**
 * @brief  SDH write data port
 *
 * @param  dataCfg: Transfer data config pointer
 *
 * @return Write length
 *
*******************************************************************************/
SDH_Stat_Type SDH_WriteDataPortBlock(SDH_Data_Cfg_Type *dataCfg)
{
    uint32_t totalLen;
    uint32_t txLen = 0U, intStatus = 0U;
    SDH_Stat_Type error = SDH_STAT_SUCCESS;

    /* Make sure the buffer size is 4 byte aligned */
    if (dataCfg->blockSize % sizeof(uint32_t) != 0U) {
        dataCfg->blockSize += sizeof(uint32_t) - (dataCfg->blockSize % sizeof(uint32_t));
    }

    totalLen = (dataCfg->blockCount * dataCfg->blockSize);

    //printk("totalLen:%d\n", totalLen);
    
    while ((error == SDH_STAT_SUCCESS) && (txLen < totalLen)) {

	while (!(intStatus & (SDH_INT_BUFFER_WRITE_READY | SDH_INT_DATA_ERRORS | SDH_INT_TUNE_ERROR))) 
	{
		uint32_t present_status = SDH_GetPresentStatus();
		intStatus = SDH_GetIntStatus();
		if(present_status&SDH_PRESENT_FLAG_BUF_WRITE_ENABLE)
		{
			break;
		}
	}

        if ((intStatus & SDH_INT_TUNE_ERROR) != 0U) {
            SDH_ClearIntStatus(SDH_INT_TUNE_ERROR);
            /* if tuning error occur ,return directly */
            error = SDH_STAT_TUNE_ERROR;
        } else if ((intStatus & SDH_INT_DATA_ERRORS) != 0U) {
            if (!(dataCfg->enableIgnoreError)) {
                error = SDH_STAT_RECV_DATA_FAIL;
            }

            /* clear data error flag */
            SDH_ClearIntStatus(SDH_INT_DATA_ERRORS);
        }

        if (error == SDH_STAT_SUCCESS) {
            txLen += SDH_WriteDataPort(dataCfg);
            /* clear buffer read ready */
            SDH_ClearIntStatus(SDH_INT_BUFFER_WRITE_READY);
            intStatus = 0U;
        }
    }

    while (!(intStatus & (SDH_INT_DATA_COMPLETED | SDH_INT_DATA_ERRORS))) {
        intStatus = SDH_GetIntStatus();
    }

    if ((intStatus & SDH_INT_DATA_ERRORS) != 0U) {
        if (!(dataCfg->enableIgnoreError)) {
            error = SDH_STAT_RECV_DATA_FAIL;
        }

        /* clear data error flag */
        SDH_ClearIntStatus(SDH_INT_DATA_ERRORS);
    }

    SDH_ClearIntStatus(SDH_INT_DATA_COMPLETED);

    return error;
}

/****************************************************************************/ /**
 * @brief  SDH transfer data
 *
 * @param  dataCfg: Data config pointer
 * @param  enDMA: Whether enable DMA
 *
 * @return command deal status
 *
*******************************************************************************/
SDH_Stat_Type SDH_TransferDataBlocking(SDH_Data_Cfg_Type *dataCfg, uint8_t enDMA)
{
    SDH_Stat_Type stat = SDH_STAT_SUCCESS;
    uint32_t intStatus = 0U;

    //printk("enDMA:%d\n", enDMA);
    if (enDMA) {
        /* Wait dataCfg complete or encounters error. */
        while (!(intStatus & (SDH_INT_DATA_COMPLETED | SDH_INT_DATA_ERRORS |
                              SDH_INT_DMA_ERROR | SDH_INT_TUNE_ERROR))) {
            intStatus = SDH_GetIntStatus();
        }

        if ((intStatus & SDH_INT_TUNE_ERROR) != 0U) {
            stat = SDH_STAT_TUNE_ERROR;
        } else if ((intStatus & (SDH_INT_DATA_ERRORS | SDH_INT_DMA_ERROR)) != 0U) {
            if ((!(dataCfg->enableIgnoreError)) || (intStatus & SDH_INT_DATA_TIMEOUT)) {
                stat = SDH_STAT_SEND_DATA_FAIL;
            }
        }

        /* load dummy data */
        if ((dataCfg->dataType == SDH_TRANS_DATA_BOOT_CONT) && (stat == SDH_STAT_SUCCESS)) {
            *(dataCfg->rxData) = bootDummy;
        }

        SDH_ClearIntStatus(SDH_INT_DATA_COMPLETED | SDH_INT_DATA_ERRORS |
                           SDH_INT_DMA_ERROR | SDH_INT_TUNE_ERROR);
    } else {
	if(dataCfg->rxData)
		;//printk("dataCfg->rxData:%px", dataCfg->rxData);
        if (dataCfg->rxData) {
            stat = SDH_ReadDataPortBlock(dataCfg);
        } else {
            stat = SDH_WriteDataPortBlock(dataCfg);
        }
    }

    return stat;
}


/****************************************************************************/ /**
 * @brief  SDH enable interrupt
 *
 * @param  dataCfg: Transfer data config pointer
 *
 * @return SDH status
 *
*******************************************************************************/
SDH_Stat_Type SDH_ConfigDataTranfer(SDH_Data_Cfg_Type *dataCfg)
{
    uint16_t tmpVal;

    if (dataCfg != NULL) {
        /* For transfer boot continous, only need to set the CREQ bit */
        if (dataCfg->dataType == SDH_TRANS_DATA_BOOT_CONT) {
            tmpVal = BL_RD_REG16(SDH_BASE_ADDR, SDH_SD_BLOCK_GAP_CTRL);
            /* Clear stop at block gap request */
            tmpVal = BL_CLR_REG_BIT(tmpVal, SDH_STOP_AT_BLOCK_GAP_REQ);
            /* Continous transfer data */
            tmpVal = BL_SET_REG_BIT(tmpVal, SDH_CONT_REQ);
            BL_WR_REG16(SDH_BASE_ADDR, SDH_SD_BLOCK_GAP_CTRL, tmpVal);

            return SDH_STAT_SUCCESS;
        }

        tmpVal = BL_RD_REG16(SDH_BASE_ADDR, SDH_SD_PRESENT_STATE_1);

        /* Check data inhibit flag */
        if (BL_IS_REG_BIT_SET(tmpVal, SDH_CMD_INHIBIT_DAT)) {
            return SDH_STAT_BUSY;
        }

        tmpVal = BL_RD_REG16(SDH_BASE_ADDR, SDH_SD_TRANSFER_MODE);

        /* Check transfer block count */
        if ((dataCfg->blockCount > SDH_MAX_BLOCK_COUNT) || ((dataCfg->txData == NULL) && (dataCfg->rxData == NULL))) {
            return SDH_STAT_INVLAID_PARA;
        }

        if (dataCfg->rxData) {
            tmpVal = BL_SET_REG_BIT(tmpVal, SDH_TO_HOST_DIR);
        } else {
            tmpVal = BL_CLR_REG_BIT(tmpVal, SDH_TO_HOST_DIR);
        }


        tmpVal = BL_SET_REG_BITS_VAL(tmpVal, SDH_AUTO_CMD_EN, 0x00);

        /* auto command 23, auto send set block count cmd before multiple read/write */
        if (dataCfg->enableAutoCommand23) {
            tmpVal = BL_SET_REG_BITS_VAL(tmpVal, SDH_AUTO_CMD_EN, 0x02);
            BL_WR_REG(SDH_BASE_ADDR, SDH_SD_SYS_ADDR_LOW, dataCfg->blockCount);
        } else if (dataCfg->enableAutoCommand12) {
            /* auto command 12 */
            tmpVal = BL_SET_REG_BITS_VAL(tmpVal, SDH_AUTO_CMD_EN, 0x01);
        }

        if (dataCfg->blockCount > 1U) {
            tmpVal = BL_SET_REG_BIT(tmpVal, SDH_MULTI_BLK_SEL);
        } else {
            tmpVal = BL_CLR_REG_BIT(tmpVal, SDH_MULTI_BLK_SEL);
        }

        /* Enable block count */
        tmpVal = BL_SET_REG_BIT(tmpVal, SDH_BLK_CNT_EN);

        BL_WR_REG16(SDH_BASE_ADDR, SDH_SD_TRANSFER_MODE, tmpVal);

        /* if transfer boot data, leave the block count to SDH_SetMmcBootConfig function */
        if (dataCfg->dataType != SDH_TRANS_DATA_BOOT) {
            /* config data block size/block count */
            BL_WR_REG16(SDH_BASE_ADDR, SDH_SD_BLOCK_SIZE, dataCfg->blockSize);
            BL_WR_REG16(SDH_BASE_ADDR, SDH_SD_BLOCK_COUNT, dataCfg->blockCount);
        }
    } else {
        /* clear data flags */
        tmpVal = BL_RD_REG16(SDH_BASE_ADDR, SDH_SD_TRANSFER_MODE);
        tmpVal = BL_SET_REG_BITS_VAL(tmpVal, SDH_AUTO_CMD_EN, 0x00);
        BL_WR_REG16(SDH_BASE_ADDR, SDH_SD_TRANSFER_MODE, tmpVal);

        tmpVal = BL_RD_REG16(SDH_BASE_ADDR, SDH_SD_PRESENT_STATE_1);

        /* Check data inhibit flag */
        if (BL_IS_REG_BIT_SET(tmpVal, SDH_CMD_INHIBIT_CMD)) {
            return SDH_STAT_BUSY;
        }
    }

    return SDH_STAT_SUCCESS;
}



/****************************************************************************/ /**
 * @brief  Disable SDH DMA
 *
 * @param  None
 *
 * @return None
 *
*******************************************************************************/
void SDH_DisableDMA(void)
{
    uint16_t tmpVal;

    tmpVal = BL_RD_REG16(SDH_BASE_ADDR, SDH_SD_TRANSFER_MODE);
    tmpVal = BL_CLR_REG_BIT(tmpVal, SDH_DMA_EN);
    BL_WR_REG16(SDH_BASE_ADDR, SDH_SD_TRANSFER_MODE, tmpVal);
}


/****************************************************************************/ /**
 * @brief  SDH set DMA config
 *
 * @param  dmaCfg: DMA config pointer
 * @param  data: Data pointer for sending
 * @param  enAutoCmd23: Whether enable auto command 23
 *
 * @return DMA config deal status
 *
*******************************************************************************/
SDH_Stat_Type SDH_SetInternalDmaConfig(SDH_DMA_Cfg_Type *dmaCfg, const uint32_t *data, uint8_t enAutoCmd23)
{
    uint16_t tmpVal;

    if (dmaCfg->dmaMode == SDH_DMA_MODE_SIMPLE) {
        /* Check DMA data buffer address align or not */
        if (((uintptr_t)data % SDH_SDMA_LENGTH_ALIGN) != 0U) {
            return SDH_STAT_DMA_ADDR_NOT_ALIGN;
        }

        /* In simple DMA mode if use auto CMD23, address should load to ADMA addr,
		     and block count should load to DS_ADDR*/
        if (enAutoCmd23) {
            BL_WR_REG(SDH_BASE_ADDR, SDH_SD_ADMA_SYS_ADDR_1, (uintptr_t)data);
        } else {
            BL_WR_REG(SDH_BASE_ADDR, SDH_SD_SYS_ADDR_LOW, (uintptr_t)data);
        }
    } else {
        /* When use ADMA, disable simple DMA */
        BL_WR_REG(SDH_BASE_ADDR, SDH_SD_SYS_ADDR_LOW, (uint32_t)0);
        BL_WR_REG(SDH_BASE_ADDR, SDH_SD_ADMA_SYS_ADDR_1, (uintptr_t)(dmaCfg->admaEntries));
    }

    /* Select DMA mode and config the burst length */
    SDH_EnableDMA(dmaCfg->dmaMode);

    /* Set DMA burst size and FIFO threshold */
    tmpVal = BL_RD_REG16(SDH_BASE_ADDR, SDH_SD_CLOCK_AND_BURST_SIZE_SETUP);
    tmpVal = BL_SET_REG_BITS_VAL(tmpVal, SDH_DMA_SIZE, dmaCfg->fifoThreshold);
    tmpVal = BL_SET_REG_BITS_VAL(tmpVal, SDH_BRST_SIZE, dmaCfg->burstSize);
    BL_WR_REG16(SDH_BASE_ADDR, SDH_SD_CLOCK_AND_BURST_SIZE_SETUP, tmpVal);

    return SDH_STAT_SUCCESS;
}


/****************************************************************************/ /**
 * @brief  Enable SDH DMA
 *
 * @param  dmaMode: DMA mode select
 *
 * @return None
 *
*******************************************************************************/
void SDH_EnableDMA(SDH_DMA_Mode_Type dmaMode)
{
    uint16_t tmpVal;

    /* Check the parameters */
    CHECK_PARAM(IS_SDH_DMA_MODE_TYPE(dmaMode));

    /* Select DMA type */
    tmpVal = BL_RD_REG16(SDH_BASE_ADDR, SDH_SD_HOST_CTRL);
    tmpVal = BL_SET_REG_BITS_VAL(tmpVal, SDH_DMA_SEL, dmaMode);
    BL_WR_REG16(SDH_BASE_ADDR, SDH_SD_HOST_CTRL, tmpVal);

    tmpVal = BL_RD_REG16(SDH_BASE_ADDR, SDH_SD_TRANSFER_MODE);
    tmpVal = BL_SET_REG_BIT(tmpVal, SDH_DMA_EN);
    BL_WR_REG16(SDH_BASE_ADDR, SDH_SD_TRANSFER_MODE, tmpVal);
}


/****************************************************************************/ /**
 * @brief  SDH create ADMA1 descriptor
 *
 * @param  adma1Entries: ADMA1 entries pointer
 * @param  maxEntries: ADMA1 entries size
 * @param  data: Data buffer to deal with
 * @param  dataLen: Data length
 * @param  flag: ADMA setting flag
 *
 * @return SDH deal status
 *
*******************************************************************************/
SDH_Stat_Type SDH_CreateADMA1Descriptor(uint32_t *adma1Entries, uint32_t maxEntries,
                                        const uint32_t *data, uint32_t dataLen, SDH_ADMA_Flag_Type flag)
{
    uint32_t miniEntries, startEntries = 0U;
    uint32_t i, dmaBufferLen = 0U;

    if (((uintptr_t)data % SDH_ADMA1_ADDRESS_ALIGN) != 0U) {
        return SDH_STAT_DMA_ADDR_NOT_ALIGN;
    }

    if (flag == SDH_ADMA_FLAG_MULTI_DESC) {
        return SDH_STAT_NOT_SUPPORT;
    }

    /* TODO:align */
    /* Make sure the buffer size is 4 byte aligned */
    if (dataLen % sizeof(uint32_t) != 0U) {
        /* make the data length as word-aligned */
        dataLen += sizeof(uint32_t) - (dataLen % sizeof(uint32_t));
    }

    /* Check if ADMA descriptor's number is enough. */
    if ((dataLen % SDH_ADMA1_DESCRIPTOR_MAX_LENGTH_PER_ENTRY) == 0U) {
        miniEntries = dataLen / SDH_ADMA1_DESCRIPTOR_MAX_LENGTH_PER_ENTRY;
    } else {
        miniEntries = ((dataLen / SDH_ADMA1_DESCRIPTOR_MAX_LENGTH_PER_ENTRY) + 1U);
    }

    /* ADMA1 needs two descriptors to finish a transfer */
    miniEntries <<= 1U;

    if (miniEntries + startEntries > maxEntries) {
        return SDH_STAT_OUTOF_RANGE;
    }

    for (i = startEntries; i < (miniEntries + startEntries); i += 2U) {
        if (dataLen > SDH_ADMA1_DESCRIPTOR_MAX_LENGTH_PER_ENTRY) {
            dmaBufferLen = SDH_ADMA1_DESCRIPTOR_MAX_LENGTH_PER_ENTRY;
        } else {
            dmaBufferLen = dataLen;
        }

        adma1Entries[i] = (dmaBufferLen << SDH_ADMA1_DESCRIPTOR_LENGTH_POS);
        adma1Entries[i] |= SDH_ADMA1_DESC_FLAG_SETLEN;
        adma1Entries[i + 1U] = (uintptr_t)(data);
        adma1Entries[i + 1U] |= SDH_ADMA1_DESC_FLAG_TRANSFER;
        data += (dmaBufferLen / sizeof(uint32_t));
        dataLen -= dmaBufferLen;
    }

    /* Set end of the descriptor */
    adma1Entries[i - 1U] |= SDH_ADMA1_DESC_FLAG_END;

    return SDH_STAT_SUCCESS;
}

/****************************************************************************/ /**
 * @brief  SDH create ADMA2 descriptor
 *
 * @param  adma2Entries: ADMA2 entries pointer
 * @param  maxEntries: ADMA2 entries size
 * @param  data: Data buffer to deal with
 * @param  dataLen: Data length
 * @param  flag: ADMA setting flag
 *
 * @return SDH deal status
 *
*******************************************************************************/
SDH_Stat_Type SDH_CreateADMA2Descriptor(SDH_ADMA2_Desc_Type *adma2Entries, uint32_t maxEntries,
                                        const uint32_t *data, uint32_t dataLen, SDH_ADMA_Flag_Type flag)
{
    uint32_t miniEntries, startEntries = 0U;
    uint32_t i, dmaBufferLen = 0U;

    if (((uintptr_t)data % SDH_ADMA2_ADDRESS_ALIGN) != 0U) {
        return SDH_STAT_DMA_ADDR_NOT_ALIGN;
    }

    /* TODO:align */
    /* Make sure the buffer size is 4 byte aligned */
    if (dataLen % sizeof(uint32_t) != 0U) {
        /* make the data length as word-aligned */
        dataLen += sizeof(uint32_t) - (dataLen % sizeof(uint32_t));
    }

    /* Check if ADMA descriptor's number is enough. */
    if ((dataLen % SDH_ADMA2_DESCRIPTOR_MAX_LENGTH_PER_ENTRY) == 0U) {
        miniEntries = dataLen / SDH_ADMA2_DESCRIPTOR_MAX_LENGTH_PER_ENTRY;
    } else {
        miniEntries = ((dataLen / SDH_ADMA2_DESCRIPTOR_MAX_LENGTH_PER_ENTRY) + 1U);
    }

    /* Calculate the start entry for multiple descriptor mode, ADMA engine is not stop, so update the descriptor
	data address and data size is enough */
    if (flag == SDH_ADMA_FLAG_MULTI_DESC) {
        for (i = 0U; i < maxEntries; i++) {
            if ((adma2Entries[i].attribute & SDH_ADMA2_DESC_FLAG_VALID) == 0U) {
                break;
            }
        }

        startEntries = i;
        /* Add one entry for dummy entry */
        miniEntries += 1U;
    }

    if (miniEntries + startEntries > maxEntries) {
        return SDH_STAT_OUTOF_RANGE;
    }

    for (i = startEntries; i < (miniEntries + startEntries); i++) {
        if (dataLen > SDH_ADMA2_DESCRIPTOR_MAX_LENGTH_PER_ENTRY) {
            dmaBufferLen = SDH_ADMA2_DESCRIPTOR_MAX_LENGTH_PER_ENTRY;
        } else {
            /* TODO:align */
            /* Adma don't support 0 data length transfer descriptor */
            dmaBufferLen = (dataLen == 0U ? sizeof(uint32_t) : dataLen);
        }

        /* Each descriptor for ADMA2 is 64-bit in length */
        adma2Entries[i].address = (dataLen == 0U) ? (uint32_t)(uintptr_t)&bootDummy : (uint32_t)(uintptr_t)data;
        adma2Entries[i].attribute = (dmaBufferLen << SDH_ADMA2_DESCRIPTOR_LENGTH_POS);
        adma2Entries[i].attribute |= (dataLen == 0U) ? 0U : (SDH_ADMA2_DESC_FLAG_TRANSFER);
        // (dataLen == 0U) ? 0U : (SDH_ADMA2_DESC_FLAG_TRANSFER | SDH_ADMA2_DESC_FLAG_INT);
        /* TODO:align */
        data += (dmaBufferLen / sizeof(uint32_t));

        if (dataLen != 0U) {
            dataLen -= dmaBufferLen;
        }
    }

    /* add a dummy valid ADMA descriptor for multiple descriptor mode, this is useful when transfer boot data, the ADMA
	engine  will not stop at block gap */
    if (flag == SDH_ADMA_FLAG_MULTI_DESC) {
        //adma2Entries[startEntries + 1U].attribute |= SDH_ADMA2_DESC_FLAG_TRANSFER;
        adma2Entries[i - 1U].attribute |= SDH_ADMA2_DESC_FLAG_TRANSFER;
    } else {
        /* Set the end bit */
        adma2Entries[i - 1U].attribute |= SDH_ADMA2_DESC_FLAG_END;
    }

    return SDH_STAT_SUCCESS;
}


/****************************************************************************/ /**
 * @brief  SDH create DMA entries according to DMA config
 *
 * @param  dmaCfg: DMA config pointer
 * @param  dataCfg: Data config pointer
 * @param  flag: ADMA create entries flag
 *
 * @return Create entries result
 *
*******************************************************************************/
SDH_Stat_Type SDH_CreateAdmaEntryConfig(SDH_DMA_Cfg_Type *dmaCfg, SDH_Data_Cfg_Type *dataCfg, SDH_ADMA_Flag_Type flag)
{
    SDH_Stat_Type stat = SDH_STAT_SUCCESS;

    /* TODO:align */
    uint32_t bootDummyOffset = (dataCfg->dataType == SDH_TRANS_DATA_BOOT_CONT ? sizeof(uint32_t) : 0);
    const uint32_t *data = (const uint32_t *)((dataCfg->rxData == NULL) ? dataCfg->txData : dataCfg->rxData);
    uint32_t dataLen = dataCfg->blockSize * dataCfg->blockCount;

    data += bootDummyOffset;
    dataLen -= bootDummyOffset;

    switch (dmaCfg->dmaMode) {
        case SDH_DMA_MODE_SIMPLE:
            stat = SDH_STAT_SUCCESS;
            break;

        case SDH_DMA_MODE_ADMA1:
            stat = SDH_CreateADMA1Descriptor(dmaCfg->admaEntries, dmaCfg->maxEntries, data, dataLen, flag);
            break;

        case SDH_DMA_MODE_ADMA2:
            stat = SDH_CreateADMA2Descriptor((SDH_ADMA2_Desc_Type *)dmaCfg->admaEntries, dmaCfg->maxEntries, data, dataLen, flag);
            break;

        default:
            return SDH_STAT_SEND_DATA_FAIL;
    }

    /* For internal dma, internal DMA configurations should not update the configurations when continous transfer the
	 * boot data, only the DMA descriptor need update */
    if ((stat == SDH_STAT_SUCCESS) && (dataCfg->dataType != SDH_TRANS_DATA_BOOT_CONT)) {
        stat = SDH_SetInternalDmaConfig(dmaCfg, data, dataCfg->enableAutoCommand23);
    }

    return stat;
}

static void SD_DecodeScr(sd_card_t *card, uint32_t *rawScr)
{
    sd_scr_t *scr;

    scr = &(card->scr);
    scr->scrStructure = (uint8_t)((rawScr[1U] & 0xF0000000U) >> 28U);
    scr->sdSpecification = (uint8_t)((rawScr[1U] & 0xF000000U) >> 24U);
	
		/*
    if ((uint8_t)((rawScr[1U] & 0x800000U) >> 23U))
    {
				//1 after erase
    }
	  */
		
    scr->sdSecurity = (uint8_t)((rawScr[1U] & 0x700000U) >> 20U);
    scr->sdBusWidths = (uint8_t)((rawScr[1U] & 0xF0000U) >> 16U);

    scr->extendedSecurity = (uint8_t)((rawScr[1U] & 0x7800U) >> 10U);
    scr->commandSupport = (uint8_t)(rawScr[1U] & 0x3U);
    scr->reservedForManufacturer = rawScr[0U];
    /* Get specification version. */  
}


/*!< SDH transfer complete callback */
void SDH_TransferFinished_CallBack(SDH_Handle_Cfg_Type *handle,SDH_Stat_Type status,void *userData)
{

	  //bflb_platform_printf("Interrupt occurs! intFlag=0x%02x,\r\n",handle->intFlag);
	  if(status != SDH_STAT_SUCCESS)
		{
			SDH_WaitStatus = SD_DataCfg_ERROR;
		}
		else
		{
			SDH_WaitStatus = SD_OK;
		}
}


/****************************************************************************/ /**
 * @brief  SDH transfer data in blocking way
 *
 * @param  dmaCfg: DMA config pointer
 * @param  transfer: Transfer config pointer
 *
 * @return Transfer deal status
 *
*******************************************************************************/
SDH_Stat_Type SDH_TransferBlocking(SDH_DMA_Cfg_Type *dmaCfg, SDH_Trans_Cfg_Type *transfer)
{
    SDH_Stat_Type stat = SDH_STAT_SUCCESS;
    SDH_CMD_Cfg_Type *cmdCfg = transfer->cmdCfg;
    SDH_Data_Cfg_Type *dataCfg = transfer->dataCfg;
    uint32_t intStatus;
    uint8_t enDMA = 1;
    uint8_t executeTuning = ((dataCfg == NULL) ? 0 : dataCfg->dataType == SDH_TRANS_DATA_TUNING);
    //volatile uint32_t tmp;
    /* Check re-tuning request */
    intStatus = SDH_GetIntStatus();

    if ((intStatus & SDH_INT_RETUNE_EVENT) != 0U) {
        SDH_ClearIntStatus(SDH_INT_RETUNE_EVENT);
        return SDH_STAT_RETUNE_REQ;
    }

    /* Update ADMA descriptor table according to different DMA mode(simple DMA, ADMA1, ADMA2).*/
    if ((dataCfg != NULL) && (dmaCfg != NULL) && (!executeTuning)) {
        stat = SDH_CreateAdmaEntryConfig(dmaCfg, dataCfg, (dataCfg->dataType & SDH_TRANS_DATA_BOOT) ? SDH_ADMA_FLAG_MULTI_DESC : SDH_ADMA_FLAG_SINGLE_DESC);
    }

    /* If the DMA desciptor configure fail or not needed , disable it */
    if ((stat != SDH_STAT_SUCCESS) || (dmaCfg == NULL)) {
        enDMA = 0;
        /* Disable DMA, using polling mode in this situation */
        SDH_DisableDMA();
    }

    /* Config the data transfer parameter */
    stat = SDH_ConfigDataTranfer(dataCfg);

    if (SDH_STAT_SUCCESS != stat) {
        return stat;
    }

    /* Send command first */
    if (dataCfg != NULL) {
        cmdCfg->flag = SDH_TRANS_FLAG_DATA_PRESENT;
    } else {
        cmdCfg->flag = 0;
    }

    SDH_SendCommand(cmdCfg);

    //printk("send cmd\n");
    /* Wait command done */
    if ((dataCfg == NULL) || (dataCfg->dataType == SDH_TRANS_DATA_NORMAL)) {
        stat = SDH_WaitCommandDone(cmdCfg);
    }
    //printk("cmd ok stat%x\n", stat);

    /*
	tmp = BL_RD_REG16(SDH_BASE,SDH_SD_ADMA_ERROR_STATUS);
	bflb_platform_printf("SDH_SD_ADMA_ERROR_STATUS->0x%x!\r\n",tmp);
	tmp = BL_RD_REG16(SDH_BASE,SDH_SD_AUTO_CMD12_ERROR_STATUS);
	*/

    /* Wait for transferring data finish */
    if ((dataCfg != NULL) && (stat == SDH_STAT_SUCCESS)) {
	//printk("in\n");
        stat = SDH_TransferDataBlocking(dataCfg, enDMA);
    }

    return stat;
}


/**
  * @brief  Allows to read blocks from a specified address  in a card.  The Data
  *         transfer can be managed by DMA mode or Polling mode. //?????
  * @note   This operation should be followed by two functions to check if the 
  *         DMA Controller and SD Card status.	   //dma????????????
  *          - SD_ReadWaitOperation(): this function insure that the DMA
  *            controller has finished all data transfer. 
  *          - SD_GetStatus(): to check that the SD Card has finished the 
  *            data transfer and it is ready for data.   
  * @param  readbuff: pointer to the buffer that will contain the received data.
  * @param  ReadAddr: Address from where data are to be read.
  * @param  BlockSize: the SD card Data block size. The Block size should be 512.
  * @param  NumberOfBlocks: number of blocks to be read.
  * @retval SD_Error: SD Card Error code.
  */
SD_Error SDH_ReadMultiBlocks(uint8_t *readbuff, uint32_t ReadAddr, uint16_t BlockSize, uint32_t NumberOfBlocks)
{
	SD_Error errorstatus = SD_OK;	
	SDH_Stat_Type stat = SDH_STAT_SUCCESS;
	if (CardType == SDIO_HIGH_CAPACITY_SD_CARD)//for sdhc block size is fixed to 512bytes 
 	{
    		BlockSize = 512;
  	}	
	
	//printk("Read-->IN, read %d blocks from %d with buffer 0x%p. \r\n",NumberOfBlocks,ReadAddr,readbuff);
	
	/*!< Set Block Size for SDSC Card,cmd16,no impact on SDHC card */
	SDH_CMD_Cfg_TypeInstance.index = SD_CMD_SET_BLOCKLEN;
	SDH_CMD_Cfg_TypeInstance.argument = (uint32_t) BlockSize;
	SDH_CMD_Cfg_TypeInstance.type = SDH_CMD_NORMAL;
	SDH_CMD_Cfg_TypeInstance.respType = SDH_RESP_R1;
	SDH_CMD_Cfg_TypeInstance.flag = SDH_TRANS_FLAG_NONE;
	
	SDH_SendCommand(&SDH_CMD_Cfg_TypeInstance);	
	stat = SDH_WaitCommandDone(&SDH_CMD_Cfg_TypeInstance);
	//printk("stat:%d response:%x\n", stat, SDH_CMD_Cfg_TypeInstance.response[0]);
	if(stat != SDH_STAT_SUCCESS){
		return SD_CMD_ERROR;
	}else if(SDH_CMD_Cfg_TypeInstance.response[0] & SD_CSR_ERRORBITS){
		return SD_CMD_ERROR;
	}	
	
	/*set cmd parameter for READ_MULTIPLE_BLOCK*/
	if(NumberOfBlocks<=1)
		SDH_CMD_Cfg_TypeInstance.index = SD_CMD_READ_SINGLE_BLOCK;
	else
		SDH_CMD_Cfg_TypeInstance.index = SD_CMD_READ_MULT_BLOCK;

	SDH_CMD_Cfg_TypeInstance.argument =  (uint32_t)ReadAddr;
	SDH_CMD_Cfg_TypeInstance.type = SDH_CMD_NORMAL;
	SDH_CMD_Cfg_TypeInstance.respType = SDH_RESP_R1;
	SDH_CMD_Cfg_TypeInstance.flag = SDH_TRANS_FLAG_DATA_PRESENT;	
	/*set data parameter for READ_MULTIPLE_BLOCK*/
	if(NumberOfBlocks<=1)
		SDH_Data_Cfg_TypeInstance.enableAutoCommand12 = DISABLE;
	else
		SDH_Data_Cfg_TypeInstance.enableAutoCommand12 = ENABLE;
	
	SDH_Data_Cfg_TypeInstance.enableAutoCommand23 = DISABLE;
	SDH_Data_Cfg_TypeInstance.enableIgnoreError = DISABLE;
	SDH_Data_Cfg_TypeInstance.dataType = SDH_TRANS_DATA_NORMAL;
	SDH_Data_Cfg_TypeInstance.blockSize = BlockSize;
	SDH_Data_Cfg_TypeInstance.blockCount = NumberOfBlocks;
	SDH_Data_Cfg_TypeInstance.rxDataLen = 0;
	SDH_Data_Cfg_TypeInstance.rxData = (uint32_t *)readbuff;
	SDH_Data_Cfg_TypeInstance.txDataLen = 0;
	SDH_Data_Cfg_TypeInstance.txData = NULL;	
	/*set parameters for SDH_DMA_Cfg_TypeInstance*/
	SDH_DMA_Cfg_TypeInstance.dmaMode = SDH_DMA_MODE_ADMA2;
	SDH_DMA_Cfg_TypeInstance.burstSize = SDH_BURST_SIZE_128_BYTES;
	SDH_DMA_Cfg_TypeInstance.fifoThreshold = SDH_FIFO_THRESHOLD_256_BYTES;
	SDH_DMA_Cfg_TypeInstance.admaEntries = (uint32_t *)adma2Entries;
	SDH_DMA_Cfg_TypeInstance.maxEntries = sizeof(adma2Entries)/sizeof(adma2Entries[0]);
	
	stat = SDH_TransferBlocking(/*&SDH_DMA_Cfg_TypeInstance*/NULL, &SDH_Trans_Cfg_TypeInstance);
	
	if(stat != SDH_STAT_SUCCESS){
		if(stat == SDH_STAT_DMA_ADDR_NOT_ALIGN)
			return SD_ADMA_ALIGN_ERROR;
		else
			return SD_DATA_ERROR;
	}
	
#if 0
	SDH_ITConfig(SDH_INT_DATA_COMPLETED|SDH_INT_DATA_ERRORS|SDH_INT_DMA_ERROR|SDH_INT_AUTO_CMD12_ERROR,ENABLE);
	
	/*wait for Xfer status. might pending here in multi-task OS*/
	//while(SDH_WaitStatus == SD_WAITING){}	
	{
		unsigned int count = 0;
		while(count++ < 2000000);
	}
	SDH_ITConfig(SDH_INT_DATA_COMPLETED|SDH_INT_DATA_ERRORS|SDH_INT_DMA_ERROR|SDH_INT_AUTO_CMD12_ERROR,DISABLE);
		
#endif
	errorstatus = SDH_STAT_SUCCESS;//SDH_WaitStatus;
	SDH_WaitStatus = SD_WAITING;
	
	return(errorstatus);		
}

SD_Error SD_SelectDeselect(uint32_t addr)
{
	SDH_Stat_Type stat = SDH_STAT_SUCCESS;
	SD_Error errorstatus = SD_OK;
	/* CMD7: select/deselect specified card */
	SDH_CMD_Cfg_TypeInstance.index = SD_CMD_SEL_DESEL_CARD;
	SDH_CMD_Cfg_TypeInstance.argument = addr;
	SDH_CMD_Cfg_TypeInstance.type = SDH_CMD_NORMAL;
	SDH_CMD_Cfg_TypeInstance.respType = SDH_RESP_R1B;
	SDH_CMD_Cfg_TypeInstance.flag = SDH_TRANS_FLAG_NONE;
	
	SDH_SendCommand(&SDH_CMD_Cfg_TypeInstance);	

	stat = SDH_WaitCommandDone(&SDH_CMD_Cfg_TypeInstance);
	if(stat != SDH_STAT_SUCCESS){
		return SD_CMD_ERROR;
	}else if(SDH_CMD_Cfg_TypeInstance.response[0] & SD_CSR_ERRORBITS){
		return SD_CMD_RESP1_ERROR;
	}	
	return errorstatus;
	
}

SD_Error FindSCR(uint16_t rca, uint32_t *pscr);
static SD_Error SDEnWideBus(BL_Fun_Type NewState)
{
  SD_Error errorstatus = SD_OK;
	SDH_Stat_Type stat = SDH_STAT_SUCCESS;
	uint32_t widthArgs=0; /*'0b00' = 1bit or '0b10'=4bit*/
  uint32_t scr[2] = {0, 0};
	
  /*!< Get SCR Register */
  errorstatus = FindSCR((uint16_t)pSDCardInfo->relativeAddress, scr);
  if (errorstatus != SD_OK)		
  {
    return(errorstatus);
  }
  /*!< If wide bus operation to be enabled */
  if (NewState == ENABLE)
		widthArgs = 0x02;
	
	if( (scr[1] & (SD_WIDE_BUS_SUPPORT|SD_SINGLE_BUS_SUPPORT) ) != SD_ALLZERO){
		
		/*!< Send CMD55 APP_CMD with argument as card's RCA */
		SDH_CMD_Cfg_TypeInstance.index = SD_CMD_APP_CMD;
		SDH_CMD_Cfg_TypeInstance.argument = (((uint32_t) pSDCardInfo->relativeAddress )<< 16);
		SDH_CMD_Cfg_TypeInstance.type = SDH_CMD_NORMAL;
		SDH_CMD_Cfg_TypeInstance.respType = SDH_RESP_R1;
		SDH_CMD_Cfg_TypeInstance.flag = SDH_TRANS_FLAG_NONE;
		
		SDH_SendCommand(&SDH_CMD_Cfg_TypeInstance);	
		stat = SDH_WaitCommandDone(&SDH_CMD_Cfg_TypeInstance);
		if(stat != SDH_STAT_SUCCESS){
			return SD_CMD_ERROR;
		}else if(SDH_CMD_Cfg_TypeInstance.response[0] & SD_CSR_ERRORBITS){
			return SD_CMD_RESP1_ERROR;
		}			
		/*!< Send ACMD6 APP_CMD with argument as 2 for wide bus mode */
		SDH_CMD_Cfg_TypeInstance.index = SD_CMD_APP_SD_SET_BUSWIDTH;
		SDH_CMD_Cfg_TypeInstance.argument = widthArgs;
		SDH_CMD_Cfg_TypeInstance.type = SDH_CMD_NORMAL;
		SDH_CMD_Cfg_TypeInstance.respType = SDH_RESP_R1;
		SDH_CMD_Cfg_TypeInstance.flag = SDH_TRANS_FLAG_NONE;
		
		SDH_SendCommand(&SDH_CMD_Cfg_TypeInstance);	
		stat = SDH_WaitCommandDone(&SDH_CMD_Cfg_TypeInstance);
		if(stat != SDH_STAT_SUCCESS){
			return SD_CMD_ERROR;
		}else if(SDH_CMD_Cfg_TypeInstance.response[0] & SD_CSR_ERRORBITS){
			return SD_CMD_RESP1_ERROR;
		}
		
	}else{
			errorstatus = SD_REQUEST_NOT_APPLICABLE;
			return(errorstatus);
	}
	return errorstatus;
}





SD_Error FindSCR(uint16_t rca, uint32_t *pscr)
{
  SD_Error errorstatus = SD_OK;
	SDH_Stat_Type stat = SDH_STAT_SUCCESS;
	uint32_t tempscr[2] = {0, 0};	
	
	if(pscr == NULL)
		return SD_INVALID_PARAMETER;
	SDH_DisableDMA();
	
	/*!< Send CMD55 APP_CMD with argument as card's RCA */
	SDH_CMD_Cfg_TypeInstance.index = SD_CMD_APP_CMD;
	SDH_CMD_Cfg_TypeInstance.argument = (((uint32_t) rca )<< 16);
	SDH_CMD_Cfg_TypeInstance.type = SDH_CMD_NORMAL;
	SDH_CMD_Cfg_TypeInstance.respType = SDH_RESP_R1;
	SDH_CMD_Cfg_TypeInstance.flag = SDH_TRANS_FLAG_NONE;
	
	SDH_SendCommand(&SDH_CMD_Cfg_TypeInstance);	
	stat = SDH_WaitCommandDone(&SDH_CMD_Cfg_TypeInstance);
	if(stat != SDH_STAT_SUCCESS){
		return SD_CMD_ERROR;
	}else if(SDH_CMD_Cfg_TypeInstance.response[0] & SD_CSR_ERRORBITS){
		return SD_CMD_RESP1_ERROR;
	}
	
	/*!< Set Block Size To 8 Bytes */ 
	SDH_Data_Cfg_TypeInstance.enableAutoCommand12 = DISABLE;
	SDH_Data_Cfg_TypeInstance.enableAutoCommand23 = DISABLE;
	SDH_Data_Cfg_TypeInstance.enableIgnoreError = DISABLE;
	SDH_Data_Cfg_TypeInstance.dataType = SDH_TRANS_DATA_NORMAL;
	SDH_Data_Cfg_TypeInstance.blockSize = 8;
	SDH_Data_Cfg_TypeInstance.blockCount = 1;
	SDH_Data_Cfg_TypeInstance.rxDataLen = 0;
	SDH_Data_Cfg_TypeInstance.rxData = tempscr;
	SDH_Data_Cfg_TypeInstance.txDataLen = 0;
	SDH_Data_Cfg_TypeInstance.txData = NULL;	
	/* Config the data transfer parameter */
	stat = SDH_ConfigDataTranfer(&SDH_Data_Cfg_TypeInstance);
	if (SDH_STAT_SUCCESS != stat){
			return SD_DataCfg_ERROR;
	}
	
	/*!< Send ACMD51 SD_APP_SEND_SCR with argument as 0 */
	SDH_CMD_Cfg_TypeInstance.index = SD_CMD_SD_APP_SEND_SCR;
	SDH_CMD_Cfg_TypeInstance.argument = 0;
	SDH_CMD_Cfg_TypeInstance.type = SDH_CMD_NORMAL;
	SDH_CMD_Cfg_TypeInstance.respType = SDH_RESP_R1;
	SDH_CMD_Cfg_TypeInstance.flag = SDH_TRANS_FLAG_DATA_PRESENT;
	
	SDH_SendCommand(&SDH_CMD_Cfg_TypeInstance);	

	stat = SDH_WaitCommandDone(&SDH_CMD_Cfg_TypeInstance);
	if(stat != SDH_STAT_SUCCESS){
		return SD_CMD_ERROR;
	}else if(SDH_CMD_Cfg_TypeInstance.response[0] & SD_CSR_ERRORBITS){
		return SD_CMD_RESP1_ERROR;
	}	
	
	/*polling data*/
	stat = SDH_TransferDataBlocking(&SDH_Data_Cfg_TypeInstance, 0);
	if (SDH_STAT_SUCCESS != stat){
			return SD_DATA_ERROR;
	}	

	*(pscr + 1) = ((tempscr[0] & SD_0TO7BITS) << 24) | ((tempscr[0] & SD_8TO15BITS) << 8) | ((tempscr[0] & SD_16TO23BITS) >> 8) | ((tempscr[0] & SD_24TO31BITS) >> 24);
  *(pscr) = ((tempscr[1] & SD_0TO7BITS) << 24) | ((tempscr[1] & SD_8TO15BITS) << 8) | ((tempscr[1] & SD_16TO23BITS) >> 8) | ((tempscr[1] & SD_24TO31BITS) >> 24);
//	SDH_MSG("scr[0]=0x%x,scr[1]=0x%x.\r\n",pscr[0],pscr[1]);
	
	pSDCardInfo->rawScr[0] = pscr[0];
	pSDCardInfo->rawScr[1] = pscr[1];
	SD_DecodeScr(pSDCardInfo,pSDCardInfo->rawScr);
	
	return errorstatus;	
}
/*
* Get SD status register
* pssr points 64bytes array
*/
static SD_Error GetSSR(uint16_t rca, uint32_t *pssr)
{
  SD_Error errorstatus = SD_OK;
	SDH_Stat_Type stat = SDH_STAT_SUCCESS;

	
	if(pssr == NULL)
		return SD_INVALID_PARAMETER;
	SDH_DisableDMA();
	
	/*!< Send CMD55 APP_CMD with argument as card's RCA */
	SDH_CMD_Cfg_TypeInstance.index = SD_CMD_APP_CMD;
	SDH_CMD_Cfg_TypeInstance.argument = (((uint32_t) rca )<< 16);
	SDH_CMD_Cfg_TypeInstance.type = SDH_CMD_NORMAL;
	SDH_CMD_Cfg_TypeInstance.respType = SDH_RESP_R1;
	SDH_CMD_Cfg_TypeInstance.flag = SDH_TRANS_FLAG_NONE;
	
	SDH_SendCommand(&SDH_CMD_Cfg_TypeInstance);	
	stat = SDH_WaitCommandDone(&SDH_CMD_Cfg_TypeInstance);
	if(stat != SDH_STAT_SUCCESS){
		return SD_CMD_ERROR;
	}else if(SDH_CMD_Cfg_TypeInstance.response[0] & SD_CSR_ERRORBITS){
		return SD_CMD_RESP1_ERROR;
	}
	
	/*!< Set Block Size To 512 Bytes */ 
	SDH_Data_Cfg_TypeInstance.enableAutoCommand12 = DISABLE;
	SDH_Data_Cfg_TypeInstance.enableAutoCommand23 = DISABLE;
	SDH_Data_Cfg_TypeInstance.enableIgnoreError = DISABLE;
	SDH_Data_Cfg_TypeInstance.dataType = SDH_TRANS_DATA_NORMAL;
	SDH_Data_Cfg_TypeInstance.blockSize = 64;
	SDH_Data_Cfg_TypeInstance.blockCount = 1;
	SDH_Data_Cfg_TypeInstance.rxDataLen = 0;
	SDH_Data_Cfg_TypeInstance.rxData = pssr;
	SDH_Data_Cfg_TypeInstance.txDataLen = 0;
	SDH_Data_Cfg_TypeInstance.txData = NULL;	
	/* Config the data transfer parameter */
	stat = SDH_ConfigDataTranfer(&SDH_Data_Cfg_TypeInstance);
	if (SDH_STAT_SUCCESS != stat){
			return SD_DataCfg_ERROR;
	}
	
	/*!< Send ACMD13 SD_APP_SEND_SCR with argument as 0 */
	SDH_CMD_Cfg_TypeInstance.index = SD_CMD_SD_APP_STAUS;
	SDH_CMD_Cfg_TypeInstance.argument = 0;
	SDH_CMD_Cfg_TypeInstance.type = SDH_CMD_NORMAL;
	SDH_CMD_Cfg_TypeInstance.respType = SDH_RESP_R1;
	SDH_CMD_Cfg_TypeInstance.flag = SDH_TRANS_FLAG_DATA_PRESENT;
	
	SDH_SendCommand(&SDH_CMD_Cfg_TypeInstance);	
	stat = SDH_WaitCommandDone(&SDH_CMD_Cfg_TypeInstance);
	if(stat != SDH_STAT_SUCCESS){
		return SD_CMD_ERROR;
	}else if(SDH_CMD_Cfg_TypeInstance.response[0] & SD_CSR_ERRORBITS){
		return SD_CMD_RESP1_ERROR;
	}	
	
	/*polling data*/
	stat = SDH_TransferDataBlocking(&SDH_Data_Cfg_TypeInstance, 0);
	if (SDH_STAT_SUCCESS != stat){
			return SD_DATA_ERROR;
	}	
	
	return errorstatus;	
}


static SDH_Cfg_Type                 SDH_Cfg_Type_Instance;

/*
 *         @arg SDIO_BusWide_8b: 8-bit data transfer
 *         @arg SDIO_BusWide_4b: 4-bit data transfer
 *         @arg SDIO_BusWide_1b: 1-bit data transfer (default)
 */
SD_Error SD_EnableWideBusOperation(SDH_Data_Bus_Width_Type WideMode)
{
  SD_Error errorstatus = SD_OK;
	
	if ((SDIO_STD_CAPACITY_SD_CARD_V1_1 == CardType) || (SDIO_STD_CAPACITY_SD_CARD_V2_0 == CardType) || (SDIO_HIGH_CAPACITY_SD_CARD == CardType))
	{
    if (SDH_DATA_BUS_WIDTH_8BITS == WideMode)   //2.0 sdh do not support 8-bit mode
    {
      errorstatus = SD_UNSUPPORTED_FEATURE;
      return(errorstatus);
    }		
    else if (SDH_DATA_BUS_WIDTH_4BITS == WideMode)
    {
			errorstatus = SDEnWideBus(ENABLE);
			if (SD_OK == errorstatus)
      {
				/* reinitialise SDH controller*/
				SDH_Cfg_Type_Instance.vlot18Enable = DISABLE;
				SDH_Cfg_Type_Instance.highSpeed = ENABLE;
				SDH_Cfg_Type_Instance.dataWidth = SDH_DATA_BUS_WIDTH_4BITS;
				SDH_Cfg_Type_Instance.volt = SDH_VOLTAGE_3P3V;
				SDH_Cfg_Type_Instance.srcClock = SDH_CLK_SRC;
				SDH_Cfg_Type_Instance.busClock = SDH_CLK_TRANSFER;
				SDH_Ctrl_Init(&SDH_Cfg_Type_Instance);				
			}
		}
		else
		{
      errorstatus = SDEnWideBus(DISABLE);
			if (SD_OK == errorstatus)
      {
				/* reinitialise SDH controller*/
				SDH_Cfg_Type_Instance.vlot18Enable = DISABLE;
				SDH_Cfg_Type_Instance.highSpeed = ENABLE;
				SDH_Cfg_Type_Instance.dataWidth = SDH_DATA_BUS_WIDTH_1BIT;
				SDH_Cfg_Type_Instance.volt = SDH_VOLTAGE_3P3V;
				SDH_Cfg_Type_Instance.srcClock = SDH_CLK_SRC;
				SDH_Cfg_Type_Instance.busClock = SDH_CLK_TRANSFER;
				SDH_Ctrl_Init(&SDH_Cfg_Type_Instance);				
			}			
		}
		
	}
	return errorstatus;	
}


SD_Error SDH_WriteMultiBlocks(uint8_t *writebuff, uint32_t WriteAddr, uint16_t BlockSize, uint32_t NumberOfBlocks)
{
  	SD_Error errorstatus = SD_OK;	
	SDH_Stat_Type stat = SDH_STAT_SUCCESS;
	if (CardType == SDIO_HIGH_CAPACITY_SD_CARD)//for sdhc block size is fixed to 512bytes 
	{
		BlockSize = 512;
	}	
	//printk("Write-->IN, write %d blocks to %d with buffer 0x%px. \r\n",NumberOfBlocks,WriteAddr,writebuff);
	
	/*!< Set Block Size for SDSC Card,cmd16,no impact on SDHC card */
	SDH_CMD_Cfg_TypeInstance.index = SD_CMD_SET_BLOCKLEN;
	SDH_CMD_Cfg_TypeInstance.argument = (uint32_t) BlockSize;
	SDH_CMD_Cfg_TypeInstance.type = SDH_CMD_NORMAL;
	SDH_CMD_Cfg_TypeInstance.respType = SDH_RESP_R1;
	SDH_CMD_Cfg_TypeInstance.flag = SDH_TRANS_FLAG_NONE;
	
	SDH_SendCommand(&SDH_CMD_Cfg_TypeInstance);	
	stat = SDH_WaitCommandDone(&SDH_CMD_Cfg_TypeInstance);
	if(stat != SDH_STAT_SUCCESS){
		return SD_CMD_ERROR;
	}else if(SDH_CMD_Cfg_TypeInstance.response[0] & SD_CSR_ERRORBITS){
		return SD_CMD_ERROR;
	}
	//printk("1.\n");	
	
	/*set cmd parameter for SD_CMD_WRITE_MULT_BLOCK*/
	if(NumberOfBlocks<=1)
		SDH_CMD_Cfg_TypeInstance.index = SD_CMD_WRITE_SINGLE_BLOCK;
	else
		SDH_CMD_Cfg_TypeInstance.index = SD_CMD_WRITE_MULT_BLOCK;
	
	SDH_CMD_Cfg_TypeInstance.argument =  (uint32_t)WriteAddr;
	SDH_CMD_Cfg_TypeInstance.type = SDH_CMD_NORMAL;
	SDH_CMD_Cfg_TypeInstance.respType = SDH_RESP_R1;
	SDH_CMD_Cfg_TypeInstance.flag = SDH_TRANS_FLAG_DATA_PRESENT;	
	/*set data parameter for WRITE_MULTIPLE_BLOCK*/
	if(NumberOfBlocks<=1)
		SDH_Data_Cfg_TypeInstance.enableAutoCommand12 = DISABLE;
	else
		SDH_Data_Cfg_TypeInstance.enableAutoCommand12 = ENABLE;
	
	SDH_Data_Cfg_TypeInstance.enableAutoCommand23 = DISABLE;
	SDH_Data_Cfg_TypeInstance.enableIgnoreError = DISABLE;
	SDH_Data_Cfg_TypeInstance.dataType = SDH_TRANS_DATA_NORMAL;
	SDH_Data_Cfg_TypeInstance.blockSize = BlockSize;
	SDH_Data_Cfg_TypeInstance.blockCount = NumberOfBlocks;
	SDH_Data_Cfg_TypeInstance.rxDataLen = 0;
	SDH_Data_Cfg_TypeInstance.rxData = NULL;
	SDH_Data_Cfg_TypeInstance.txDataLen = 0;
	SDH_Data_Cfg_TypeInstance.txData = (uint32_t *)writebuff;
	/*set parameters for SDH_DMA_Cfg_TypeInstance*/
	SDH_DMA_Cfg_TypeInstance.dmaMode = SDH_DMA_MODE_ADMA2;
	SDH_DMA_Cfg_TypeInstance.burstSize = SDH_BURST_SIZE_128_BYTES;
	SDH_DMA_Cfg_TypeInstance.fifoThreshold = SDH_FIFO_THRESHOLD_256_BYTES;
	SDH_DMA_Cfg_TypeInstance.admaEntries = (uint32_t *)adma2Entries;
	SDH_DMA_Cfg_TypeInstance.maxEntries = sizeof(adma2Entries)/sizeof(adma2Entries[0]);
	//printk("2.\n");	
	
	stat = SDH_TransferBlocking(/*&SDH_DMA_Cfg_TypeInstance*/NULL, &SDH_Trans_Cfg_TypeInstance);
	
	//printk("3.\n");	
	if(stat != SDH_STAT_SUCCESS){
		if(stat == SDH_STAT_DMA_ADDR_NOT_ALIGN)
			return SD_ADMA_ALIGN_ERROR;
		else
			return SD_DATA_ERROR;
	}

	
#if 0
	SDH_ITConfig(SDH_INT_DATA_COMPLETED|SDH_INT_DATA_ERRORS|SDH_INT_DMA_ERROR|SDH_INT_AUTO_CMD12_ERROR,ENABLE);
	
	/*wait for Xfer status. might pending here in multi-task OS*/
	//while(SDH_WaitStatus == SD_WAITING){}
	{
		unsigned int count = 0;
		while(count++ < 2000000);
	}
		
	SDH_ITConfig(SDH_INT_DATA_COMPLETED|SDH_INT_DATA_ERRORS|SDH_INT_DMA_ERROR|SDH_INT_AUTO_CMD12_ERROR,DISABLE);
		
	if(SDH_WaitStatus != SD_OK)
	{
		SDH_WaitStatus = SD_WAITING;
		return SD_DATA_ERROR;
	}
#endif	
	
	SDH_WaitStatus = SD_WAITING;	
	errorstatus = SDH_STAT_SUCCESS;//WaitInProgramming();
	
	return(errorstatus);		
}

EXPORT_SYMBOL(SDH_WriteMultiBlocks);
EXPORT_SYMBOL(SDH_ReadMultiBlocks);


void read_sd_block(unsigned long index)
{
	int ret;
	int i;
	int zero_flag = 1;
	char buf[1024] = {0};
	static char str_buf[4096] = {0};
	
	printk("buf:%px rand:%ld", buf, index);
	ret = SDH_ReadMultiBlocks(buf, index*512, 512, 1);
	printk("ret %d\n", ret);


	str_buf[0] = 0;
	for(i = 1; i <= 512; i++)
	{
		if(buf[i-1])
		{
			zero_flag = 0;
			break;
		}
	}
	if(!zero_flag)
	{
		for(i = 1; i <= 512; i++)
		{
			sprintf(str_buf+strlen(str_buf), "%02x ", buf[i-1]);
			if(i%20==0)
			{
				printk("%s\n", str_buf);
				str_buf[0] = 0;
			}

		}
	}
	printk("%s\n", str_buf);
}

void write_sd_block(unsigned long index)
{
	int i;
	int ret;
	char buf[1024] = {0};
	for(i = 0; i < 512; i++)
		buf[i] = i;
	ret = SDH_WriteMultiBlocks(buf, index*512, 512, 1);
	printk("ret=%d\n", ret);
}

static int __init shm_driver_init(void) {  
    int ret = 0;

    if(!mapped_addr)
    {
	mapped_addr = ioremap(PHYSICAL_ADDRESS, SIZE);  
    	printk("0x%x mapped_addr 0x%px\n", PHYSICAL_ADDRESS, mapped_addr); 
    }
    
    if (!mapped_addr) {  
            printk("ioremap_nocache error!\n");
	    return -ENOMEM;  
    }  

    SDH_EnableIntStatus(SDH_INT_ALL);
    printk("stat:%x\n", BL_RD_REG(SDH_BASE_ADDR, SDH_SD_PRESENT_STATE_1));
    {
	uint32_t int_status;
	SD_Error errorstatus = SD_OK;
	printk("power stat:%d\r\n", SDH_PowerON());
	SDH_ClearIntStatus(SDH_INT_ALL);
	int_status = SDH_GetIntStatus();
	printk("int status:%x\n", int_status);
	SDH_InitializeCards();
	errorstatus = SD_SelectDeselect((uint32_t) ( (gSDCardInfo.relativeAddress) << 16));
	if (errorstatus != SD_OK)
	{ /*fails at select stage*/
		//return(errorstatus);
	}

	/* setup 4-bit mode and uplift SDH_CLK to 50MHz*/
	errorstatus = SD_EnableWideBusOperation(4);
	if (errorstatus != SD_OK)
	{ /*fails at EnableWideBusOperation stage*/
		//return(errorstatus);
	}

	errorstatus = GetSSR(gSDCardInfo.relativeAddress, gSDCardInfo.rawSsr);
	if (errorstatus != SD_OK)
	{
		//return(errorstatus);
	}
	printk("sdcard blockCount:%d blockSize:%d\r\n", gSDCardInfo.blockCount,
		gSDCardInfo.blockSize);
	printk("init ok\n");
    }

    //return ret;

    ret = alloc_chrdev_region(&first_dev, 0, 1, DEVICE_NAME);  
    if (ret) {  
        printk(KERN_ERR "Failed to allocate chrdev region\n");  
        return ret;  
    }  
    cdev_init(&my_cdev, &my_driver_fops);  
    my_cdev.owner = THIS_MODULE;  
    ret = cdev_add(&my_cdev, first_dev, 1);  
    if (ret) {  
        printk(KERN_ERR "Failed to add cdev\n");  
        unregister_chrdev_region(first_dev, 1);  
        return ret;  
    }  
    my_class = class_create(THIS_MODULE, DEVICE_NAME);  
    device_create(my_class, NULL, MKDEV(MAJOR(first_dev), MINOR(first_dev)), NULL, DEVICE_NAME);  
   
    printk("shm init ok\n");
    return 0;  
}

static void __exit shm_driver_exit(void) {
    	iounmap(mapped_addr); 
	device_destroy(my_class, first_dev);
	class_destroy(my_class);
	unregister_chrdev_region(first_dev, 1);
	cdev_del(&my_cdev);
	//kfree(my_cdev);
    	printk("iounmap\n"); 
}

module_init(shm_driver_init);
module_exit(shm_driver_exit);
MODULE_LICENSE("GPL");
