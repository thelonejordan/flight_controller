#include "sam3x8e.h"
#include "InitTimer.h"
#include "InitUART.h"

int main(void)
{
    /* Initialize the SAM system */
    SystemInit();
	TIMERbegin();
	UARTbegin(57600);
	PMC->PMC_WPMR &= ~(PMC_WPMR_WPEN);      //disable write protect
	PIOC->PIO_WPMR &= ~(PIO_WPMR_WPEN);     //disable write protect
	PIOB->PIO_WPMR &= ~(PIO_WPMR_WPEN);     //disable write protect
	
	PMC->PMC_PCER0 |= (1u<<ID_PIOC);        //enable peripheral clock
	PIOC->PIO_IER |= PIO_IER_P3;            //enable pin change interrupts
	
	PIOC->PIO_PER |= PIO_PER_P3;            //enable PIO controller
	PIOC->PIO_ODR |= PIO_ODR_P3;            //set as input
	PIOC->PIO_PUER |= PIO_PUER_P3;          //pull up  the pin
	
    PIOB->PIO_PER |= PIO_PER_P27;           //enable PIO controller
	PIOB->PIO_OER |= PIO_OER_P27;	        //set as output
	
	NVIC_EnableIRQ(PIOC_IRQn);              //configure NVIC
	
    while (1) 
    {
		UARTprintln("outside");
	}
}

void PIOC_Handler(){
	UARTprintln('A');
	if(PIOC->PIO_PDSR & PIO_PDSR_P3) PIOB->PIO_CODR |= PIO_CODR_P27;
	
	else PIOB->PIO_SODR |= PIO_SODR_P27;
	
	uint32_t status = PIOC->PIO_ISR;
}