#ifndef InitTIMER
#define InitTIMER

#include "sam3x8e.h"

void TIMERbegin(){
	//Disable Watchdog Timer
	WDT->WDT_MR |= WDT_MR_WDDIS;
    //Configure PMC
	PMC->PMC_WPMR &= ~(PMC_WPMR_WPEN);                                  //Disable the Write Protect Mode
	PMC->PMC_PCER0 |= PMC_PCER0_PID27;                                  //Enable TC0 Channel 0 Peripheral Clock

	TC0->TC_WPMR &= ~(TC_WPMR_WPEN);                                    //Disable the Write Protect Mode
	TC0->TC_CHANNEL[0].TC_CMR |= TC_CMR_TCCLKS_TIMER_CLOCK3;            //Set Clock Source to MCK/32
	TC0->TC_CHANNEL[0].TC_CMR |= TC_CMR_WAVE | TC_CMR_WAVSEL_UPDOWN;    //Set Wave select

	TC0->TC_CHANNEL[0].TC_CCR |= TC_CCR_CLKEN | TC_CCR_SWTRG;           //Enable Clock and start counter
}

void _delay_us(unsigned long us){
	us *= 2.625;                                           //calculate ticks to wait
	uint32_t counter = TC0->TC_CHANNEL[0].TC_CV;          //Read current value from Counter Value Register
	while ((TC0->TC_CHANNEL[0].TC_CV - counter) < us);    //wait
}

void _delay_ms(unsigned long ms){
	ms *= 2625;                                          //calculate ticks to wait
	uint32_t counter = TC0->TC_CHANNEL[0].TC_CV;          //Read current value from Counter Value Register
	while ((TC0->TC_CHANNEL[0].TC_CV - counter) < ms);    //wait
} 

uint32_t micros(){
	uint32_t counter = TC0->TC_CHANNEL[0].TC_CV;          //Read current value from Counter Value Register
    counter /= 2.625;                                       
	return counter;
}

uint32_t millis(){
	uint32_t counter = TC0->TC_CHANNEL[0].TC_CV;          //Read current value from Counter Value Register
	counter /= 2625;
	return counter;
}


#endif