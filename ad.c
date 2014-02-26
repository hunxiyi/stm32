/** 
 *  @file		ad.c
 *	@brief	压力传感器驱动程序
 *	@par		Copyright (c):  
 *          
 *  @par		硬件连接:
 *  -        GPIOC_4   -- Sencer
 *  -        GPIOA_12  -- EN
 *  -        GPIOA_11  -- A1
 *  -        GPIOA_8   -- A0 
 *  -        GPIOC_12  -- 3V_EN
 *	@par		修改日志:
 *					版本			  日期           作者      说明
 */

#include "ad.h"
#include "delay.h"
#include "eeprom.h"
#include "rtc.h"
#include "spi_flash.h"


/** 引脚定义 */
#define AD_Sencer_1_PORT 				GPIOC
#define AD_Sencer_2_PORT 				GPIOC
#define AD_Sencer_3_PORT 				GPIOB
#define AD_Sencer_Power_PORT		    GPIOC
												
#define AD_Sencer_1_RCC 				RCC_AHBPeriph_GPIOC
#define AD_Sencer_2_RCC 				RCC_AHBPeriph_GPIOC
#define AD_Sencer_3_RCC 				RCC_AHBPeriph_GPIOB
#define AD_Sencer_Power_RCC			    RCC_AHBPeriph_GPIOC

#define AD_Sencer_1_PIN 				GPIO_Pin_4
#define AD_Sencer_2_PIN 				GPIO_Pin_5
#define AD_Sencer_3_PIN 				GPIO_Pin_0
#define AD_Sencer_Power_PIN			    GPIO_Pin_12



#define AD_BUFFER_SIZE     		10

/** ad采样缓冲区 */
static __IO uint16_t adc_buffer[AD_BUFFER_SIZE*3];

/**
 *  AD转换器配置函数
 *  @param  None
 *  @return None
 */
void ad_config(void)
{	
	GPIO_InitTypeDef GPIO_InitStructure;
	DMA_InitTypeDef  DMA_InitStructure;
	ADC_InitTypeDef  ADC_InitStructure;  	

	RCC_AHBPeriphClockCmd(AD_Sencer_1_RCC | AD_Sencer_2_RCC
	                    | AD_Sencer_3_RCC | AD_Sencer_Power_RCC, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin   = AD_Sencer_Power_PIN;	    /**< AD供电开关 */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
 	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_Init(AD_Sencer_Power_PORT, &GPIO_InitStructure);		
	ad_power_switch(ENABLE);									/**< AD供电使能 */
	
    GPIO_InitStructure.GPIO_Pin   = AD_Sencer_1_PIN;				
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;		 
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(AD_Sencer_1_PORT, &GPIO_InitStructure);	
  
	GPIO_InitStructure.GPIO_Pin  = AD_Sencer_2_PIN;					
    GPIO_Init(AD_Sencer_2_PORT, &GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Pin  = AD_Sencer_3_PIN;					
	GPIO_Init(AD_Sencer_3_PORT, &GPIO_InitStructure);
	
	RCC_HSICmd(ENABLE);														     /**< 注意：此芯片的ADC时钟由HSI提供 */
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	DMA_DeInit(DMA1_Channel1);                                                   /**< 配置通道 */
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;              /**< "桥梁"的一端，AD采样值 */
	DMA_InitStructure.DMA_MemoryBaseAddr     = (uint32_t)&adc_buffer;            /**< "桥梁"的另一端，内存存储基地址 */
	DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralSRC;            /**< 单向传输 */
	DMA_InitStructure.DMA_BufferSize         = AD_BUFFER_SIZE * 3;               /**< 缓存大小 */
	DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;        /**< 关闭外设指针的自动递增 */
	DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;             /**< 开启内存指针的自动递增 */
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;  /**< 外设数据宽度 */
	DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;      /**< 内存数据宽度 */
	DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;                /**< 循环缓存     */
	DMA_InitStructure.DMA_Priority           = DMA_Priority_High;                /**< 优先级       */
	DMA_InitStructure.DMA_M2M                = DMA_M2M_Disable;                  /**< 禁用memory to memory */
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	DMA_Cmd(DMA1_Channel1, ENABLE);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	ADC_InitStructure.ADC_Resolution           = ADC_Resolution_12b;             /**< 分辨率 12位 */
	ADC_InitStructure.ADC_ScanConvMode         = ENABLE;                         /**< 关闭扫描模式，只有1个通道时，一般关闭 */
	ADC_InitStructure.ADC_ContinuousConvMode   = ENABLE;                         /**< 开启连续转换模式 */
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;  /**< 关闭外部触发，使用软件触发（ADC_SoftwareStartConvCmd） */
	ADC_InitStructure.ADC_DataAlign            = ADC_DataAlign_Right;            /**< 12位数据右对齐 */
	ADC_InitStructure.ADC_NbrOfConversion      = 3;                              /**< 开启通道数，3个 */
	ADC_Init(ADC1, &ADC_InitStructure);
	
	/** 采样周期设为：采样时间：384cycles, 转化时间：12cycles, 总时间：(12+384)/16M = 30us; */ 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 1, ADC_SampleTime_384Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 2, ADC_SampleTime_384Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8,  3, ADC_SampleTime_384Cycles);
	
	ADC_DMACmd(ADC1, ENABLE);
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
	ADC_Cmd(ADC1, ENABLE); 

	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_RCNR) == SET);     /**< Regular channel not ready */
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_ADONS) == RESET);  /**< ADC ON status */
	ADC_SoftwareStartConv(ADC1);                             /**< 软件触发 */
}	

/**
 *  AD电源开关
 *  @param [in]NewState 开关控制
 *  @return None
 *  
 */
void ad_power_switch(FunctionalState NewState)
{
	if(NewState != DISABLE)
		GPIO_ResetBits(AD_Sencer_Power_PORT, AD_Sencer_Power_PIN);
	else
		GPIO_SetBits(AD_Sencer_Power_PORT, AD_Sencer_Power_PIN);	
}

/**
 * @brief 得到采样值，每10个数去除1个最大值和1个最小值剩余8个值求和
 *
 * @param adc15 读出存储AD值的指针adc15
 */
void ad_get_data(uint16_t *adc15)
{
	uint8_t i, j;
	uint16_t ad[3][AD_BUFFER_SIZE], max, min;	
	uint16_t sum;
	
	for(i = 0; i < 3; i++)
	{
		/** 初始化 */
		sum = 0;
		max = 0;
		min = 0xffff;	
		
		/** 提取AD采样值，同时得到1个最大值和1个最小值*/
		for(j = 0; j < AD_BUFFER_SIZE; j++)
		{
			ad[i][j] = adc_buffer[j * 3 + i];			
			
			sum += ad[i][j];	
			
			if(ad[i][j] > max)	  /*取最大值*/
			{
				max = ad[i][j];
			}
			if(ad[i][j] < min)	  /*取最小值*/
			{
				min = ad[i][j];
			}
		}		
		/** 去除1个最大值和1个最小值*/
		sum -= min;	
		sum -= max;	
		
        /** 得到15位AD值（8个12位数之和） */
		*(adc15 + i) = sum;
	}
}	
	
	
/**
 * @brief 将采样值转换为传感器读数(扩大了10倍)
 *
 * @param value 转换后的数值指针
 * @param adc15 传入的AD值指针
 */
void ad_get_value(uint16_t *convert_value, const uint16_t *adc15)
{
	uint8_t i;
	uint16_t eeprom_value[6];
    uint16_t eeprom_adc15_cal_l[3];
    uint16_t eeprom_adc15_cal_h[3];
	
	/** 读出保存在eeprom中的低点、高点标定值 */	
	eeprom_read(ADDR_CAL_L, 6, &eeprom_value[0]);

    for(i = 0; i < 3; i++)
    {
         eeprom_adc15_cal_l[i] = eeprom_value[i];
         eeprom_adc15_cal_h[i] = eeprom_value[i + 3];
    }
	
	/** 得到传感器值 */		
	for(i = 0; i < 3; i++)
	{				
		/** 采样值大于零点标定值时，*/
		if(*(adc15 + i) > eeprom_adc15_cal_l[i])			  
		{								   
            *(convert_value + i) = (uint16_t)( FULL_SCALE
            * (*(adc15 + i) - eeprom_adc15_cal_l[i]) 
            / (eeprom_adc15_cal_h[i] - eeprom_adc15_cal_l[i]));
		}
		/** 采样值小于零点标定值时，*/
		else				   				
		{
			*(convert_value + i) = 0;					   
		} 			
	}	
}	

/**
 *  存储flash数据
 *  @param  value
 *  @return None
 */
void ad_store_flash(uint16_t *convert_value)
{
	uint8_t i;
	uint8_t buf[12];
	uint32_t flash_wr_addr;  /**< flash写地址 */
	
	/** 读取实时钟(6 byte) */
	rtc_rd_calendar(&buf[0]);
	/** 读取传感器值 */
	for(i = 0; i < 3; i++)
	{
		buf[2 * i + 6] = (*(convert_value + i)) / 256;  /**< 传感器数据高八位在前 */
		buf[2 * i + 7] = (*(convert_value + i)) % 256;
	}	
	
	/** 更新flash写指针 */
	flash_wr_addr = RTC_ReadBackupRegister(RTC_BKP_flash_wr_addr);  /**< 读出这次要写入的地址 */

    /** 当从第一扇区开始写，或者如果写一帧数据(12 bytes)超过最后一个扇区，
         则将第一扇区擦除并从第一扇区开始写 */
    if((flash_wr_addr + 12) > 0x1FFFFF)
    {
        spi_falsh_specify_sector_erase(0);
	    flash_wr_addr = 0;
    }
    /** 如果写一帧数据(12 bytes)达到下一扇区，则将下一扇区进行擦除 */
    else if(((flash_wr_addr + 12) & 0xFF0000) > (flash_wr_addr & 0xFF0000))
    {
        spi_falsh_specify_sector_erase(((flash_wr_addr & 0xFF0000) >> 16) + 1);        
    }

    SPI_FLASH_BufferWrite(buf, flash_wr_addr, 12);
    
	flash_wr_addr += 12;  /**< 下次要写入的地址，一帧数据12 bytes */
	RTC_WriteBackupRegister(RTC_BKP_flash_wr_addr, (uint32_t)flash_wr_addr);	
}


