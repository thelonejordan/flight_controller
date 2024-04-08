#ifndef InitUART
#define InitUART

#include <math.h>
#include "sam3x8e.h"

#define BIN 2
#define OCT 8
#define DEC 10
#define HEX 16
#define RES 4



char CHR[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
uint8_t res = 0;

void UARTbegin(unsigned long baud_rate){
    //Disable the Write Protect Mode
    PMC->PMC_WPMR &= ~(PMC_WPMR_WPEN);                                  
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

void writeByte(char c){
	
	while(!(UART->UART_SR & UART_SR_TXRDY));       //Wait till last character is sent and THR is empty
    UART->UART_THR = c;                            //Write to UART_THR
}

void write(char c){
	writeByte(c);
}

void write(const char* str){
	while(*str) writeByte(*str++);
}

void writeln(char c){
	write(c);
	writeByte('\n');
}

void writeln(const char* str){
	write(str);
	writeByte('\n');
}

void newline(void){
	writeByte('\n');
}

void tabspace(void){
	writeByte('\t');
}

void getrem(long num, unsigned short base = DEC)
{
	if(num>0){
		getrem(num/base, base);
		int rem = num%base;
		writeByte(CHR[rem]);
	}
}

void getremf(long num, int res = RES)
{
	if(res>0){
		res--;
		getremf(num/10, res);
		int rem = num%10;
		writeByte(CHR[rem]);
	}
}

template <typename T>
void print(T val,uint8_t base = DEC, uint8_t res = RES){
	if(val == 0) writeByte(CHR[0]);
	
	else if(val>0){
		long num = val;
		double temp = (val - num)*pow(10, res);
		long dec = temp + pow(0.1, res);
		if(num==0) writeByte(CHR[0]);
		else getrem(num, base);
		if(dec>0){
			writeByte('.');
			getremf(dec, res);
		}
	}
	
	else if(val<0){
		long num = val;
		double temp = (num - val)*pow(10, res);
		long dec = temp + pow(0.1, res);
		writeByte('-');
		num *= -1;
		if(num==0) writeByte(CHR[0]);
		else getrem(num);
		if(dec>0){
			writeByte('.');
			getremf(dec, res);
		}
	}
}

template <typename R>
void println(R val, uint8_t base = DEC, uint8_t res = RES){
	print(val, base, res);
	newline();
}

#endif