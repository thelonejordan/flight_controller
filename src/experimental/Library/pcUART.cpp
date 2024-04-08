#include "sam3x8e.h"
#include "pcUART.h"

void Serial::begin(unsigned long baud_rate){
    //Enable UART Peripheral Clock
	PMC->PMC_PCER0 |= PMC_PCER0_PID8;
	//Initialize RX and TX pins
	PIOA->PIO_PDR |= PIO_PDR_P8 | PIO_PDR_P9;
	//Disable PDC Channel
	UART->UART_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS ;
	//Reset and disable receiver and transmitter
	UART->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX | UART_CR_RXDIS | UART_CR_TXDIS ;
	//Configure Mode
	UART->UART_MR |= UART_MR_PAR_NO | UART_MR_CHMODE_NORMAL;            //No Parity and normal ch mode
	//Configure Rate Generator
    UART->UART_BRGR |= UART_BRGR_CD(5250000/baud_rate);                 //84000000/16 = 5250000
	// Configure interrupts
	UART->UART_IDR = 0xFFFFFFFF;
	UART->UART_IER = UART_IER_RXRDY | UART_IER_OVRE | UART_IER_FRAME;
	/*// Enable UART interrupt in NVIC
    NVIC_EnableIRQ(_dwIrq);*/

	// Enable receiver and transmitter
	UART->UART_CR = UART_CR_RXEN | UART_CR_TXEN ;
}

void Serial::WRITE(char c){
    UART->UART_THR = c;                            //Write to UART_THR
	while(!(UART->UART_SR & UART_SR_TXRDY));       //Wait till character is sent and THR is empty
}

void Serial::getrem(long num, unsigned short base = DEC)
{
	if(num>0){
        getrem(num/base, base);
        int rem = num%base;
        WRITE(CHR[rem]);
    }
}

void Serial::print(long num, unsigned short base = DEC){
	if(num == 0) WRITE(CHR[0]);
	else if(num<0){
    	num *= -1;
    	WRITE('-');
    	getrem(num, base);
    }
    else if(num>0){
    	getrem(num, base);
    }     
}

void Serial::print(char str){
	WRITE(str[i]);
}

void Serial::print(char str[]){
	unsigned int i = 0;
	while(str[i] != NULL){
		WRITE(str[i]);
		i++;
	}
}

void Serial::println(long num, unsigned short base = DEC){
    print(num, base);
    WRITE('\n');
}

void Serial::println(char str){
	print(str);
	WRITE('\n');
}

void Serial::println(char str[]){
	print(str);
	WRITE('\n');
}