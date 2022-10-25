/*
 * Name: main.c
 * Author: You
 *
 * This is a stub. Write your code here.
 *
 */
#include "MKL25Z4.h"
#include "wdog.h"
#include "led.h"
#include "btn.h"
#include "littleHelper.h"

void blockingFunctionExample();
void nonBlockingFunctionExample();

int main(void)
{
	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);
	led_init();
	btn_init();

	// povol prijem preruseni pro port A a nastav prioritu na 2
	NVIC_SetPriority(PORTA_IRQn,2);
	NVIC_EnableIRQ(PORTA_IRQn);
	// povol generovani preruseni v periferii portA pro sestupnou i nástupnou hranu
	changeMultipleBitInRegister(PORTA->PCR[4],11u,PORT_PCR_IRQC_MASK,PORT_PCR_IRQC_SHIFT);

	while (1) {
		wdog_refresh();
		// zavolej funkci
		blockingFunctionExample();
	}

	return 0;
}

void PORTA_IRQHandler(void)
{
	// vynuluj flag
	PORTA->PCR[4] |= 1 << PORT_PCR_ISF_SHIFT;
	// zavolej funkci
	nonBlockingFunctionExample();
}


void blockingFunctionExample()
{
	for(int i = 0;i<1000000;i++){}
	// zapni diodu 1 a vypni diodu 2
	GPIOB->PDOR |= 1<<8;
	GPIOB->PDOR &= ~(1<<9);
	for(int i = 0;i<1000000;i++){}
	// zapni diodu 2 a vypni diodu 1
	GPIOB->PDOR |= 1<<9;
	GPIOB->PDOR &= ~(1<<8);

}

void nonBlockingFunctionExample()
{
	// pokud je tlacitko 2 sepnute rozsvit diodu 3
	if(GPIOA->PDIR & (1<<4))
	{
		GPIOB->PDOR |= 1<<10;
	}
	else
	{
		GPIOB->PDOR &= ~(1<<10);
	}
}


