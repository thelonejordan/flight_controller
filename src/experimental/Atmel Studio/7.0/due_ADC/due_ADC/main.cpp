/*
 * due_ADC.cpp
 *
 * Created: 12-06-2017 19:57:29
 * Author : Jyotirmaya Mahanta
 */ 


#include "sam3x8e.h"
#include "InitTimer.h"
#include "InitUART.h"

float volt = 678;

int main(void)
{
    SystemInit();
	UARTbegin(57600);
	TIMERbegin();
	
    //Configure PMC
    PMC->PMC_WPMR &= ~(PMC_WPMR_WPEN);                                  //Disable the Write Protect Mode
    PMC->PMC_PCER1 |= PMC_PCER1_PID37;                                  //Enable ADC Peripheral Clock

    PIOA->PIO_PDR |= PIO_PDR_P16;                                       //Disable PIO Controller
	
	ADC->ADC_WPMR &= ~(ADC_WPMR_WPEN);                                  //Disable the Write Protect Mode
	ADC->ADC_CHER |= ADC_CHER_CH7;                                      //Enable A0 pin
	ADC->ADC_MR = 0;
	//ADC->ADC_MR = ADC_MR_PRESCAL(2);                                    //ADC Clock set to 16MHz
	ADC->ADC_MR = ADC_MR_PRESCAL(4);                                    //ADC Clock set to 8MHz
	ADC->ADC_MR |= ADC_MR_TRACKTIM(3);
	ADC->ADC_MR |= ADC_MR_STARTUP_SUT8;
	ADC->ADC_EMR = 0;
	
	
	/*
	When a conversion is completed, the resulting 12-bit digital value is stored in the Channel Data Register
	(ADC_CDRx) of the current channel
    */
	
    while (1) 
    {
		ADC->ADC_CR |= ADC_CR_START;                                        //Software Trigger
		volt = ADC->ADC_CDR[7];
		UARTprintln(volt);
		_delay_ms(4);
    }
}
