/**
  ******************************************************************************
  * @file    PWR/STANDBY/main.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main program body.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "main.h"


/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup PWR_STANDBY
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
USART_InitTypeDef USART_InitStructure;
static __IO uint32_t uwTimingDelay;
RCC_ClocksTypeDef 	RCC_ClockFreq;


/*PWM variable*/
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
uint16_t TIM1_CH1Pulse = 0;
uint16_t TIM1_Period = 0;	
/* Private function prototypes -----------------------------------------------*/

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(USART3, (uint8_t) ch);

  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
  {}

  return ch;
}  
/* Private functions ---------------------------------------------------------*/
void PWM_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*
		TIM1 Frequency 	= TIM1 counter clock/(ARR + 1)
						= =72 MHz / (7200 -1) = 10 KHz
						
		TIM1 Channel1 duty cycle = (TIM3_CCR1/ TIM3_ARR)* 100 = 35%
	*/

	TIM1_Period = (uint16_t) (SystemCoreClock / 10000) - 1;
	TIM1_CH1Pulse = (uint16_t) (((uint32_t) 35 * (TIM1_Period - 1)) / 100);
	
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = TIM1_Period;			// 10 000 Hz
	TIM_TimeBaseStructure.TIM_Prescaler = (1 -1);
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	/* PWM1 Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = TIM1_CH1Pulse;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;	
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);

	/* TIM1 counter enable */
	TIM_Cmd(TIM1, ENABLE);

	/* TIM1 Main Output Enable */
	TIM_CtrlPWMOutputs(TIM1, ENABLE);

	
}


/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
void Delay(__IO uint32_t mTime)
{ 
	uwTimingDelay = mTime;
	while(uwTimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (uwTimingDelay != 0x00)
  { 
    uwTimingDelay--;
  }
}


void UART_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable GPIO clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);


	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); 

	/* Configure USART Tx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);


	/* Configure USART Rx as input floating */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; 


	/* USART configuration */
	USART_Init(USART3, &USART_InitStructure);

	/* Enable USART */
	USART_Cmd(USART3, ENABLE);
}

/**
  * @brief  Configures the SysTick to generate an interrupt each 1000 ms.
  * @param  None
  * @retval None
  */
void SysTick_Configuration(void)
{
	RCC_GetClocksFreq(&RCC_ClockFreq);
	
	#if 1 //debug
	printf("===========================\r\n");
	printf("SYSCLK_Frequency = %d \r\n" , 	RCC_ClockFreq.SYSCLK_Frequency);
	printf("HCLK_Frequency = %d \r\n" , 		RCC_ClockFreq.HCLK_Frequency);
	printf("PCLK_Frequency = %d \r\n" , 		RCC_ClockFreq.PCLK1_Frequency);
	printf("ADCCLK_Frequency= %d \r\n" , 	RCC_ClockFreq.PCLK2_Frequency);
	#endif /*debug*/

	/* Setup SysTick Timer for 1ms interrupts  */
	if (SysTick_Config(SystemCoreClock / 1000))
	{ 
		/* Capture error */ 
		while (1);
	}

	/* Select AHB clock(HCLK) divided by 8 as SysTick clock source */
//	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);

	/* Set SysTick Preemption Priority to 1 */
	NVIC_SetPriority(SysTick_IRQn, 0x04);
}

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
	static uint16_t i = 0;	
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */     

	UART_Config();

	SysTick_Configuration();

	PWM_Config();

	/* Output a message on Hyperterminal using printf function */
	printf("\n\rUSART Printf Example: retarget the C library printf function to the USART\n\r");

	while (1)
	{
		Delay(1000);
		printf("Polling log (%4d)\r\n",++i);
	}
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}

#endif

/**
  * @}
  */

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
