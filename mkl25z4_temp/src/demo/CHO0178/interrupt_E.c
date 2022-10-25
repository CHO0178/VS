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

	// povol generovani preruseni v periferii port A pro sestupnou i nástupnou hranu


	while (1) {
		wdog_refresh();
		// zavolej funkci

	}

	return 0;
}

void PORTA_IRQHandler(void)
{
	// vynuluj flag

	// zavolej funkci

}


void blockingFunctionExample()
{
	for(int i = 0;i<1000000;i++){}
	// zapni diodu 1 a vypni diodu 2

	for(int i = 0;i<1000000;i++){}
	// zapni diodu 2 a vypni diodu 1


}

void nonBlockingFunctionExample()
{
	// pokud je tlacitko 2 sepnute rozsvit diodu 3

}


