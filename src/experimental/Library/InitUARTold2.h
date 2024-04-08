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
bool flag = true;  //val has not come yet
unsigned short res = 0;
long expp = pow(DEC, RES);

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

void UARTWRITE(char c){
    UART->UART_THR = c;                            //Write to UART_THR
	while(!(UART->UART_SR & UART_SR_TXRDY));       //Wait till character is sent and THR is empty
}

void UARTgetrem(long num, unsigned short base = DEC, bool choice = false)
{
    if(choice){
        if(num>0 || res>1){
            res--;
            UARTgetrem(num/base, base, true);
            int rem = num%base;
            UARTWRITE(CHR[rem]);
        }
    }
    else {
    	if(num>0){
            UARTgetrem(num/base, base);
            int rem = num%base;
            UARTWRITE(CHR[rem]);
        }
    }
}

void UARTgetremf(long num)
{
	unsigned short base = DEC;
    int rem = num%base;
    while(flag){
        if(rem == 0){
            num /= base;
            rem = num%base;
            res--;
        }
        else{
            flag = false;
        }
    }
	if(num>0){
        UARTgetrem(num/base, base, true);
        rem = num%base;
        UARTWRITE(CHR[rem]);
    }
}

void UARTprint(double val, unsigned short base = DEC){
	flag = true;
    res = RES;
    long num = val;
    if(num == 0) UARTWRITE(CHR[0]);
	else if(num<0){
    	num *= -1;
    	UARTWRITE('-');
    	UARTgetrem(num, base);
    }
    else if(num>0){
    	UARTgetrem(num, base);
    }     
    
    val *= expp;                     // 10^RES
    num = val - expp*num;            // 10^RES
    if(num>0)UARTWRITE('.');
    if(num>0){
    	UARTgetremf(num);
    }     
}

void UARTprint(char str){
	UARTWRITE(str);
}

void UARTprint(char str[]){
	unsigned int i = 0;
	while(str[i] != '\0'){
		UARTWRITE(str[i]);
		i++;
	}
}

void UARTprintln(double val, unsigned short base = DEC){
    UARTprint(val, base);
    UARTWRITE('\n');
}

void UARTprintln(char str){
	UARTprint(str);
	UARTWRITE('\n');
}

void UARTprintln(char str[]){
	UARTprint(str);
	UARTWRITE('\n');
}

#endif