
#include "contiki.h"
#include "dev/leds.h"

void leds_arch_init(void){
	P8DIR |= BIT2 | BIT3 | BIT4;
	P8OUT &= ~(BIT2 | BIT3 | BIT4);
}

unsigned char leds_arch_get(void){
	return ((P8OUT & BIT2) ? 0 : LEDS_RED)
			| ((P8OUT & BIT3) ? 0 : LEDS_GREEN)
			| ((P8OUT & BIT4) ? 0 : LEDS_BLUE);
}

void leds_arch_set(unsigned char leds){

	if(leds & LEDS_RED) P8OUT |= BIT2;
	else P8OUT &= ~BIT2;

	if(leds & LEDS_GREEN) P8OUT |= BIT3;
	else P8OUT &= ~BIT3;

	if(leds & LEDS_BLUE) P8OUT |= BIT4;
	else P8OUT &= ~BIT4;

}
