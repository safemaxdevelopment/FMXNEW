/*	BASIC INTERRUPT VECTOR TABLE FOR STM8 devices
 *	Copyright (c) 2007 STMicroelectronics
 */
#include "stm8s_gpio.h"
#include "stm8s_tim4.h"
#include "stm8s_tim2.h"

typedef void @far (*interrupt_handler_t)(void);

extern void TimingDelay_Decrement(void);
extern volatile uint16_t pulse_count;
extern volatile int one_second_elapsed;

//extern @far @interrupt void TIM2_UPD_Interrupt_Handler(void);

struct interrupt_vector {
	unsigned char interrupt_instruction;
	interrupt_handler_t interrupt_handler;
};

@far @interrupt void NonHandledInterrupt (void)
{
	/* in order to detect unexpected events during development, 
	   it is recommended to set a breakpoint on the following instruction
	*/
	return;
}

// Define your custom interrupt handler for IRQ 6
@far @interrupt void CustomIRQ6Handler(void)
{
				pulse_count++;
	#if 0
	 int i;
	 for(i=0; i<10000; i++); // some delay
				 for(i=0; i<10000; i++); // some delay
				 GPIO_WriteReverse(GPIOB, GPIO_PIN_5); //
				  for(i=0; i<10000; i++); // some delay
					for(i=0; i<10000; i++); // some
	#endif				
}



@far @interrupt void TIM4_UPD_Interrupt_Handler(void)
{
	 // TimingDelay_Decrement();
   // GPIO_WriteReverse(GPIOB, GPIO_PIN_5); // Toggle PB5
   // TIM4_ClearITPendingBit(TIM4_IT_UPDATE); // Clear interrupt flag
}



extern void _stext();     /* startup routine */

struct interrupt_vector const _vectab[] = {
	{0x82, (interrupt_handler_t)_stext}, /* reset */
	{0x82, NonHandledInterrupt}, /* trap  */
	{0x82, NonHandledInterrupt}, /* irq0  */
	{0x82, NonHandledInterrupt}, /* irq1  */
	{0x82, NonHandledInterrupt}, /* irq2  */
	{0x82, NonHandledInterrupt}, /* irq3  */
	{0x82, NonHandledInterrupt}, /* irq4  */
	{0x82, NonHandledInterrupt}, /* irq5  */
	{0x82, CustomIRQ6Handler},   /* irq6  - Custom handler for IRQ 6 */
	{0x82, NonHandledInterrupt}, /* irq7  */
	{0x82, NonHandledInterrupt}, /* irq8  */
	{0x82, NonHandledInterrupt}, /* irq9  */
	{0x82, NonHandledInterrupt}, /* irq10 */
	{0x82, NonHandledInterrupt}, /* irq11 */
	{0x82, NonHandledInterrupt}, /* irq12 */
		{0x82, NonHandledInterrupt}, /* irq14 */
	{0x82, NonHandledInterrupt}, /* irq15 */
	{0x82, NonHandledInterrupt}, /* irq16 */
	{0x82, NonHandledInterrupt}, /* irq17 */
	{0x82, NonHandledInterrupt}, /* irq18 */
	{0x82, NonHandledInterrupt}, /* irq19 */
	{0x82, NonHandledInterrupt}, /* irq20 */
	{0x82, NonHandledInterrupt}, /* irq21 */
	{0x82, NonHandledInterrupt}, /* irq22 */
	{0x82, TIM4_UPD_Interrupt_Handler}, /* irq23 */
	{0x82, NonHandledInterrupt}, /* irq24 */
	{0x82, NonHandledInterrupt}, /* irq25 */
	{0x82, NonHandledInterrupt}, /* irq26 */
	{0x82, NonHandledInterrupt}, /* irq27 */
	{0x82, NonHandledInterrupt}, /* irq28 */
	{0x82, NonHandledInterrupt}, /* irq29 */
};