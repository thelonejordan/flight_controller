


#include "sam3x8e.h"

void _delay_us(unsigned long us){
	                                                      //calculate ticks to wait
	uint32_t counter = TC0->TC_CHANNEL[0].TC_CV;          //Read current value from Counter Value Register
	while ((TC0->TC_CHANNEL[0].TC_CV - counter) < us);    //wait
}

void _delay_ms(unsigned long ms){
	ms *= 10500;                                          //calculate ticks to wait
	uint32_t counter = TC0->TC_CHANNEL[0].TC_CV;          //Read current value from Counter Value Register
	while ((TC0->TC_CHANNEL[0].TC_CV - counter) < ms);    //wait
} 

uint32_t micros(){
	uint32_t counter = TC0->TC_CHANNEL[0].TC_CV;          //Read current value from Counter Value Register
    counter /= 10.5;                                       
	return counter;
}

uint32_t millis(){
	uint32_t counter = TC0->TC_CHANNEL[0].TC_CV;          //Read current value from Counter Value Register
	counter /= 10500;
	return counter;
}

int main(void)
{
    SystemInit();

    //Configure PMC
	PMC->PMC_WPMR &= ~(PMC_WPMR_WPEN);                                  //Disable the Write Protect Mode
	PMC->PMC_PCER0 |= PMC_PCER0_PID27;                                  //Enable TC0 Channel 0 Peripheral Clock

	TC0->TC_WPMR &= ~(TC_WPMR_WPEN);                                    //Disable the Write Protect Mode
	TC0->TC_CHANNEL[0].TC_CMR |= TC_CMR_TCCLKS_TIMER_CLOCK2;            //Set Clock Source to MCK/8
	TC0->TC_CHANNEL[0].TC_CMR |= TC_CMR_WAVE | TC_CMR_WAVSEL_UPDOWN;    //Set Wave select

	TC0->TC_CHANNEL[0].TC_CCR |= TC_CCR_CLKEN | TC_CCR_SWTRG;           //Enable Clock and start counter
	
	PIOB->PIO_PER |= PIO_PER_P27;
	PIOB->PIO_OER |= PIO_OER_P27;

    while (1)
    {
	    PIOB->PIO_SODR |= PIO_SODR_P27;        //Set Output Data Register
		_delay_ms(1000);
	    PIOB-> PIO_CODR |= PIO_CODR_P27;       //Clear Output Data Register
	    _delay_ms(1000);
    }
}
