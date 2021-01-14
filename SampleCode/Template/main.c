/*************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    A project template for M480 MCU.
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define LED_R						(PH0)
#define LED_Y						(PH1)
#define LED_G						(PH2)

#define MASTER_DATA_NUM			(16)	//(512)

#define BridgeSpiPortNum				(SPI0)

#define SPI_GET_SSLINE_FLAG(spi)		(((spi)->STATUS & SPI_STATUS_SSLINE_Msk) >> SPI_STATUS_SSLINE_Pos)

#define SPI_GET_SSINAIF_FLAG(spi)		(((spi)->STATUS & SPI_STATUS_SSINAIF_Msk) >> SPI_STATUS_SSINAIF_Pos)
#define SPI_SET_SSINAIF_FLAG(spi)		((spi)->STATUS |= SPI_STATUS_SSINAIF_Msk)

#define SPI_GET_SSACTIF_FLAG(spi)		(((spi)->STATUS & SPI_STATUS_SSACTIF_Msk) >> SPI_STATUS_SSACTIF_Pos)
#define SPI_SET_SSACTIF_FLAG(spi)		((spi)->STATUS |= SPI_STATUS_SSACTIF_Msk)

uint8_t buffer[MASTER_DATA_NUM] = {0};
uint8_t TestCount = 0;
uint8_t RxData = 0;
uint32_t TxDataCount = 0;
uint32_t RxDataCount = 0;

uint32_t conter_tick = 0;

typedef enum{
	flag_DEFAULT = 0 ,

	flag_SPI_RX_ready ,
	
	flag_END	
}Flag_Index;

volatile uint32_t BitFlag = 0;
#define BitFlag_ON(flag)							(BitFlag|=flag)
#define BitFlag_OFF(flag)							(BitFlag&=~flag)
#define BitFlag_READ(flag)							((BitFlag&flag)?1:0)
#define ReadBit(bit)								(uint32_t)(1<<bit)

#define is_flag_set(idx)							(BitFlag_READ(ReadBit(idx)))
#define set_flag(idx,en)							( (en == 1) ? (BitFlag_ON(ReadBit(idx))) : (BitFlag_OFF(ReadBit(idx))))

void tick_counter(void)
{
	conter_tick++;
}

uint32_t get_tick(void)
{
	return (conter_tick);
}

void set_tick(uint32_t t)
{
	conter_tick = t;
}

void dump_buffer(uint8_t *pucBuff, int nBytes)
{
    uint16_t i = 0;
    
    printf("dump_buffer : %2d\r\n" , nBytes);    
    for (i = 0 ; i < nBytes ; i++)
    {
        printf("0x%2X," , pucBuff[i]);
        if ((i+1)%8 ==0)
        {
            printf("\r\n");
        }            
    }
    printf("\r\n\r\n");
}

void SPI_Slave_Process(void)
{
	#if 1	// send data at next clock in same packet
	volatile uint8_t flag_time_out = 0;
	volatile uint32_t counter_time_out = 0;
//	static uint8_t u8Data = 0;
	
	do
	{
		//printf("wait for SPI_SSACT_INT_MASK\r\n");
	}while(SPI_GetIntFlag(BridgeSpiPortNum ,SPI_SSACT_INT_MASK ) == 0);
	SPI_ClearIntFlag(BridgeSpiPortNum , SPI_SSACT_INT_MASK);

	SPI_ClearTxFIFO(BridgeSpiPortNum);
	SPI_ClearRxFIFO(BridgeSpiPortNum);

	TxDataCount = 0;
	RxDataCount = 0;

	while(SPI_GetIntFlag(BridgeSpiPortNum,SPI_SSINACT_INT_MASK) == 0)
	{
		flag_time_out = 0 ;
		counter_time_out = 0xFFFF;
		do
		{
			if (SPI_GET_RX_FIFO_EMPTY_FLAG(BridgeSpiPortNum) == 0)
			{
				break;
			}

			if (counter_time_out-- == 0)
			{
				flag_time_out = 1;
				break;
			}

		}while(1);

		if (flag_time_out)
		{
			break;
		}

//		u8Data = SPI_READ_RX(BridgeSpiPortNum);		
		buffer[RxDataCount++] = SPI_READ_RX(BridgeSpiPortNum);
		
	    if (SPI_GET_TX_FIFO_FULL_FLAG(BridgeSpiPortNum) == 0)
	    {
//	        SPI_WRITE_TX(BridgeSpiPortNum, u8Data);	
			SPI_WRITE_TX(BridgeSpiPortNum, buffer[TxDataCount++]);
	    }		
	};

	if (SPI_GetIntFlag(BridgeSpiPortNum,SPI_SSINACT_INT_MASK))
	{
		SPI_ClearIntFlag(BridgeSpiPortNum , SPI_SSINACT_INT_MASK);
		TxDataCount = 0;
		RxDataCount = 0;
		
        if (SPI_GET_TX_FIFO_FULL_FLAG(BridgeSpiPortNum) == 0)
        {
            SPI_WRITE_TX(BridgeSpiPortNum, 0xFF);    /* Write to TX FIFO */
        }
	}
		
	#else


	#if 1	//below 2 method , same result , send data packet , at next transmission
	
	do
	{
		//printf("wait for SPI_GET_SSACTIF_FLAG\r\n");
	}while(SPI_GET_SSACTIF_FLAG(BridgeSpiPortNum) == 0);
	SPI_SET_SSACTIF_FLAG(BridgeSpiPortNum);

	SPI_ClearTxFIFO(BridgeSpiPortNum);
	SPI_ClearRxFIFO(BridgeSpiPortNum);

	TxDataCount = 0;
	RxDataCount = 0;

	while(SPI_GET_SSINAIF_FLAG(BridgeSpiPortNum) == 0)
	{	
		if (SPI_GET_RX_FIFO_EMPTY_FLAG(BridgeSpiPortNum) == 0)
		{
			buffer[RxDataCount++] = SPI_READ_RX(BridgeSpiPortNum);
		}
		
	    if ((SPI_GET_TX_FIFO_FULL_FLAG(BridgeSpiPortNum) == 0) && (TxDataCount < MASTER_DATA_NUM))
	    {
	        SPI_WRITE_TX(BridgeSpiPortNum, buffer[TxDataCount++]);
	    }				
	};

	if (SPI_GET_SSINAIF_FLAG(BridgeSpiPortNum))
	{
		SPI_SET_SSINAIF_FLAG(BridgeSpiPortNum);
		TxDataCount = 0;
		RxDataCount = 0;
		
        if (SPI_GET_TX_FIFO_FULL_FLAG(BridgeSpiPortNum) == 0)
        {
            SPI_WRITE_TX(BridgeSpiPortNum, 0xFF);    /* Write to TX FIFO */
        }
	}

	#else
	SPI_ClearTxFIFO(BridgeSpiPortNum);
	SPI_ClearRxFIFO(BridgeSpiPortNum);

	TxDataCount = 0;
	RxDataCount = 0;

    while(RxDataCount < MASTER_DATA_NUM)
    {
        /* Check TX FULL flag and TX data count */
        if((SPI_GET_TX_FIFO_FULL_FLAG(BridgeSpiPortNum) == 0) && (TxDataCount < MASTER_DATA_NUM))
            SPI_WRITE_TX(BridgeSpiPortNum, buffer[TxDataCount++]); /* Write to TX FIFO */
        /* Check RX EMPTY flag */
        if(SPI_GET_RX_FIFO_EMPTY_FLAG(BridgeSpiPortNum) == 0)
            buffer[RxDataCount++] = SPI_READ_RX(BridgeSpiPortNum); /* Read RX FIFO */
    }	
	#endif
	
	#endif

}

void SPI_Init(void)
{
	SYS_ResetModule(SPI0_RST);
    SPI_Open(BridgeSpiPortNum, SPI_SLAVE, SPI_MODE_0, 8, (uint32_t)NULL);
	
	SPI_ClearRxFIFO(BridgeSpiPortNum);
	SPI_ClearTxFIFO(BridgeSpiPortNum);
	SPI_SetFIFO(BridgeSpiPortNum,2,2);

    SPI_WRITE_TX(BridgeSpiPortNum, 0xFF);    /* Dummy Write to prevent TX under run */

}


void TMR1_IRQHandler(void)
{	
//	static uint32_t log = 0;	
	
    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        TIMER_ClearIntFlag(TIMER1);
		tick_counter();
		
		if ((get_tick() % 1000) == 0)
		{
//			printf("%s : %2d\r\n" , __FUNCTION__ , log++);
			LED_Y ^= 1;
		}		
    }
}

void TIMER1_HW_Init(void)
{
    CLK_EnableModuleClock(TMR1_MODULE);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HIRC, 0);
}

void TIMER1_Init(void)
{
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);	
    TIMER_Start(TIMER1);
}

void TIMER0_HW_Init(void)
{
	CLK_EnableModuleClock(TMR0_MODULE);
	CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK0, 0);
}

void TIMER0_Polling(uint32_t u32Usec)
{
	TIMER_Delay(TIMER0, u32Usec);
}

void LED_Init(void)
{
	GPIO_SetMode(PH,BIT0,GPIO_MODE_OUTPUT);
	GPIO_SetMode(PH,BIT1,GPIO_MODE_OUTPUT);
	GPIO_SetMode(PH,BIT2,GPIO_MODE_OUTPUT);
	
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable External XTAL (4~24 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk | CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk | CLK_STATUS_HIRCSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(FREQ_192MHZ);
    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART clock source from HXT */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));

    CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL2_SPI0SEL_PCLK1, MODULE_NoMsk);
    CLK_EnableModuleClock(SPI0_MODULE);

//    CLK_EnableModuleClock(PDMA_MODULE);

//	TIMER0_HW_Init();
	TIMER1_HW_Init();
	
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Setup SPI0 multi-function pins */
    SYS->GPA_MFPL |= SYS_GPA_MFPL_PA0MFP_SPI0_MOSI | SYS_GPA_MFPL_PA1MFP_SPI0_MISO | SYS_GPA_MFPL_PA2MFP_SPI0_CLK | SYS_GPA_MFPL_PA3MFP_SPI0_SS;

    /* Enable SPI0 clock pin (PA2) schmitt trigger */
    PA->SMTEN |= GPIO_SMTEN_SMTEN2_Msk;

    /* Enable SPI0 I/O high slew rate */
    GPIO_SetSlewCtl(PA, 0xF, GPIO_SLEWCTL_HIGH);
	
    /* Lock protected registers */
    SYS_LockReg();
}

/*
 * This is a template project for M480 series MCU. Users could based on this project to create their
 * own application without worry about the IAR/Keil project settings.
 *
 * This template application uses external crystal as HCLK source and configures UART0 to print out
 * "Hello World", users may need to do extra system configuration based on their system design.
 */

int main()
{

    SYS_Init();
    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

	printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
	printf("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());	
	printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());

	LED_Init();
	TIMER1_Init();
	SPI_Init();

    /* Got no where to go, just loop forever */
    while(1)
    {
		SPI_Slave_Process();
	
    }
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
