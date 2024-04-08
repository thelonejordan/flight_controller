/*
 * due_i2c.cpp
 *
 * Created: 01-06-2017 20:22:37
 * Author : Jyotirmaya Mahanta
 */ 


#include "sam3x8e.h"


int main(void)
{

    SystemInit();
	int data = 0;
    //Disable PIO Controller
    PIOB->PIO_PDR |= PIO_PDR_P12 | PIO_PDR_P13;          
	//Peripheral A selected by default
	//Disable the Write Protect Mode
	PMC->PMC_WPMR &= ~(PMC_WPMR_WPEN);                                  
	//Enable TWI peripheral Clock
	PMC->PMC_PCER0 |= PMC_PCER0_PID23;
	//Wave Generator - Set TWI Clock to 100kHz
	TWI1->TWI_CWGR = 0;
	TWI1->TWI_CWGR = TWI_CWGR_CKDIV(1)|TWI_CWGR_CHDIV(0xD4)|TWI_CWGR_CLDIV(0xD4);
	//Wave Generator - Set TWI Clock to 400kHz
	TWI1->TWI_CWGR = 0;
	TWI1->TWI_CWGR = TWI_CWGR_CKDIV(0)|TWI_CWGR_CHDIV(0x65)|TWI_CWGR_CLDIV(0x65);
	//SVDIS: Disable the slave mode. MSEN: Enable the master mode.
	TWI1->TWI_CR |= TWI_CR_SVDIS | TWI_CR_MSEN;
	TWI1->TWI_MMR = 0;
	TWI1->TWI_MMR |= TWI_MMR_DADR(0x68);
	
	/*Read a single byte 
	//The 'device address' is used to access slave, set Transfer Direction Bit to 1
	TWI1->TWI_MMR |= TWI_MMR_IADRSZ_1_BYTE | TWI_MMR_MREAD;
	//The Internal Address
	TWI1->TWI_IADR = TWI_IADR_IADR(0x75);
	//Read Single Byte
	TWI1->TWI_CR |= TWI_CR_START | TWI_CR_STOP;
	//Read Status register, wait until RXRDY is 1
	while(!(TWI1->TWI_SR & TWI_SR_RXRDY));
	//Read Receive Holding register
	data = TWI1->TWI_RHR;
	//Read Status Register
	while(!(TWI1->TWI_SR & TWI_SR_TXCOMP));	*/
    
	/*Read multiple  byte 
	//The 'device address' is used to access slave, set Transfer Direction Bit to 1
	TWI1->TWI_MMR |= TWI_MMR_IADRSZ_1_BYTE | TWI_MMR_MREAD;
	//The Internal Address
	TWI1->TWI_IADR = TWI_IADR_IADR(0x00);
	//START
	TWI1->TWI_CR |= TWI_CR_START;
	int numofbytes = 0;
	for(int i = 0; i<numofbytes-1; i++){
		//Read Status register, wait until RXRDY is 1
		while(!(TWI1->TWI_SR & TWI_SR_RXRDY));
		//Read Receive Holding register
		data = TWI1->TWI_RHR;
	}
	//STOP
	TWI1->TWI_CR |= TWI_CR_STOP;
	//Read Status register, wait until RXRDY is 1
	while(!(TWI1->TWI_SR & TWI_SR_RXRDY));
	//Read Receive Holding register
	data = TWI1->TWI_RHR;
	//Read Status Register
	while(!(TWI1->TWI_SR & TWI_SR_TXCOMP));*/
	
	/*Write a single byte
	//The 'device address' is used to access slave, set Transfer Direction Bit to 1
	TWI1->TWI_MMR |= TWI_MMR_IADRSZ_1_BYTE;
	TWI1->TWI_MMR &= ~(TWI_MMR_MREAD);
	//The Internal Address
	TWI1->TWI_IADR = TWI_IADR_IADR(0x00);
	//Load data to be sent
	TWI1->TWI_THR = 0x00;
	//STOP
	TWI1->TWI_CR |= TWI_CR_STOP;
	//Read Status register, wait until TXRDY is 1
	while(!(TWI1->TWI_SR & TWI_SR_TXRDY));
	//Read Status Register
	while(!(TWI1->TWI_SR & TWI_SR_TXCOMP));
	*/
	
	
	
    while (1) 
    {
    }
}
