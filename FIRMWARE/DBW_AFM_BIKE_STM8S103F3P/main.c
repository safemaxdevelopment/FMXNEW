#include "stm8s.h"
#include "stm8s_clk.h"
#include "stm8s_exti.h"
#include "stm8s_gpio.h"
#include "stm8s_itc.h"
#include "stm8s_tim4.h"
#include "stm8s_tim2.h"


#define PULSE_LIMIT 197

volatile uint16_t pulse_count = 0;  // Pulse counter
uint16_t pulse_copy_count = 0;
volatile int one_second_elapsed=0;
__IO uint32_t TimingDelay = 0;

void CLK_Config(void) {
    CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1); // Set system clock to 16MHz
}

void GPIO_Config(void) {
    // Configure PD2 as input with interrupt
		
#if 0		
		typedef enum {
  EXTI_SENSITIVITY_FALL_LOW  = (uint8_t)0x00, /*!< Interrupt on Falling edge and Low level */
  EXTI_SENSITIVITY_RISE_ONLY = (uint8_t)0x01, /*!< Interrupt on Rising edge only */
  EXTI_SENSITIVITY_FALL_ONLY = (uint8_t)0x02, /*!< Interrupt on Falling edge only */
  EXTI_SENSITIVITY_RISE_FALL = (uint8_t)0x03  /*!< Interrupt on Rising and Falling edges */
} EXTI_Sensitivity_TypeDef;
#endif


    GPIO_Init(GPIOD, GPIO_PIN_2, GPIO_MODE_IN_FL_IT);
    EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOD, EXTI_SENSITIVITY_FALL_ONLY); // Rising and falling edge detection

    // Configure PB5 as output (LED)
    GPIO_Init(GPIOB, GPIO_PIN_5, GPIO_MODE_OUT_PP_LOW_FAST);
    GPIO_WriteHigh(GPIOB, GPIO_PIN_5); // Initialize LED to OFF (assuming active low)
		
		GPIO_Init(GPIOC, GPIO_PIN_3, GPIO_MODE_OUT_PP_LOW_FAST);
    GPIO_WriteHigh(GPIOC, GPIO_PIN_3); // Initialize LED to OFF (assuming active low)
}

void TIM2_Config(void)
{
    TIM2_TimeBaseInit(TIM2_PRESCALER_1024, 15624); // (16MHz / 1024) = 15625 Hz
                                                   // (1s = 15625-1)
    TIM2_ITConfig(TIM2_IT_UPDATE, ENABLE); // Enable TIM2 update interrupt
    TIM2_Cmd(ENABLE); // Start TIM2

}



void main(void) {
    
    enableInterrupts(); // Enable global interrupts
		
		
		#if 1
		while (1) {
			
			   //GPIO_WriteReverse(GPIOB, GPIO_PIN_5);
			
			   if(one_second_elapsed)
				 {
					  one_second_elapsed=0;
						pulse_copy_count=pulse_count;
						pulse_count=0;
						
						if (pulse_copy_count>PULSE_LIMIT)
						{
							 GPIO_WriteHigh(GPIOC, GPIO_PIN_3);//relay
							 GPIO_WriteLow(GPIOB, GPIO_PIN_5);//led
						}
						
						else
						{
							 GPIO_WriteLow(GPIOC, GPIO_PIN_3);//relay
							 GPIO_WriteHigh(GPIOB, GPIO_PIN_5);//led
							
						}
						
				 }
			
				 //Delay(50);
			
		}
		#endif

}

void Delay(__IO uint32_t nTime)
{
  TimingDelay = nTime;

  while (TimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  {
    TimingDelay--;
  }
}
