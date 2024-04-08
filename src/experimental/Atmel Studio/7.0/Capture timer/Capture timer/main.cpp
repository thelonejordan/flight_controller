#include "sam3x8e.h"


int main(void)
{
    SystemInit();
	WDT->WDT_MR |= WDT_MR_WDDIS;
	
	PMC->PMC_WPMR &= ~(PMC_WPMR_WPEN);                                  //Disable the Write Protect Mode
	PMC->PMC_PCER0 |= (PMC_PCER0_PID27);                                  //Enable TC0 Channel 0 Peripheral Clock

	TC0->TC_WPMR &= ~(TC_WPMR_WPEN);                                    //Disable the Write Protect Mode
	TC0->TC_CHANNEL[1].TC_CMR = 0;
	TC0->TC_CHANNEL[1].TC_CMR |= TC_CMR_TCCLKS_TIMER_CLOCK2;            //Set Clock Source to MCK/8
	TC0->TC_CHANNEL[1].TC_CMR |= TC_CMR_WAVE | TC_CMR_WAVSEL_UPDOWN;    //Set Wave select

	TC0->TC_CHANNEL[1].TC_CCR |= TC_CCR_CLKEN | TC_CCR_SWTRG;           //Enable Clock and start counter
}
	
	
    while (1) 
    {
    }
}
