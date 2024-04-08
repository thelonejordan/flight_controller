#ifndef InitTWI
#define InitTWI
#include "sam3x8e.h"
#define LOW_SPEED_CKDIV    1       //100kHz
#define LOW_SPEED_CHDIV  (0xD4)    //100kHz
#define LOW_SPEED_CLDIV  (0xD4)    //100kHz
#define HIGH_SPEED_CKDIV   0       //400kHz
#define HIGH_SPEED_CHDIV (0x65)    //400kHz
#define HIGH_SPEED_CLDIV (0x65)    //400kHz
 
uint8_t buffer[15];
unsigned int length_of_buffer = 0;
unsigned int index = 0;

void TWIbegin(uint8_t twi_ckdiv = LOW_SPEED_CKDIV, uint8_t twi_chdiv = LOW_SPEED_CHDIV, uint8_t twi_cldiv = LOW_SPEED_CLDIV){
	//Disable the Write Protect Mode
	PMC->PMC_WPMR &= ~(PMC_WPMR_WPEN);
	PIOB->PIO_WPMR &= ~(PIO_WPMR_WPEN);
	//Disable PIO Controller
	PIOB->PIO_PDR |= PIO_PDR_P12|PIO_PDR_P13;
	//Peripheral A selected by default
	PIOB->PIO_ABSR &= ~(PIO_ABSR_P12|PIO_ABSR_P13);
	//Enable TWI peripheral Clock
	PMC->PMC_PCER0 |= (1u<<ID_TWI1);
	//Wave Generator - Set TWI Clock to 100kHz or 400kHz
	TWI1->TWI_CWGR = 0;
	TWI1->TWI_CWGR = TWI_CWGR_CKDIV(twi_ckdiv)|TWI_CWGR_CHDIV(twi_chdiv)|TWI_CWGR_CLDIV(twi_cldiv);
	//SVDIS: Disable the slave mode. MSEN: Enable the master mode.
	TWI1->TWI_CR |= TWI_CR_SVDIS | TWI_CR_MSEN;
	TWI1->TWI_MMR = 0;
}

void TWIbeginTransmission(uint8_t address){
	
	TWI1->TWI_MMR |= TWI_MMR_DADR(address);
}

void TWIendTransmission(){
	TWI1->TWI_MMR = 0;
}

uint8_t TWIread(){
	if(index<length_of_buffer){
		return buffer[index];
		index++;
	}
	else{
		index = 0;
		return buffer[index];
	}
}

void TWIwrite(uint8_t i_address, uint8_t value){
	/*Write a single byte*/
	//The 'device address' is used to access slave, set Transfer Direction Bit to 1
	TWI1->TWI_MMR |= TWI_MMR_IADRSZ_1_BYTE;
	TWI1->TWI_MMR &= ~(TWI_MMR_MREAD);
	//The Internal Address
	TWI1->TWI_IADR = TWI_IADR_IADR(i_address);
	//Load data to be sent
	TWI1->TWI_THR = value;
	//STOP
	TWI1->TWI_CR |= TWI_CR_STOP;
	//Read Status register, wait until TXRDY is 1
	while(!(TWI1->TWI_SR & TWI_SR_TXRDY));
	//Read Status Register
	while(!(TWI1->TWI_SR & TWI_SR_TXCOMP));
}

void TWIwrite2(uint8_t i_address_1, uint8_t i_address_2, uint8_t value){
	/*Write a single byte*/
	//The 'device address' is used to access slave, set Transfer Direction Bit to 1
	TWI1->TWI_MMR |= TWI_MMR_IADRSZ_2_BYTE;
	TWI1->TWI_MMR &= ~(TWI_MMR_MREAD);
	//The Internal Address
	TWI1->TWI_IADR = TWI_IADR_IADR(i_address_1<<8|i_address_2);
	//Load data to be sent
	TWI1->TWI_THR = value;
	//STOP
	TWI1->TWI_CR |= TWI_CR_STOP;
	//Read Status register, wait until TXRDY is 1
	while(!(TWI1->TWI_SR & TWI_SR_TXRDY));
	//Read Status Register
	while(!(TWI1->TWI_SR & TWI_SR_TXCOMP));
}

void TWIrequestFrom(uint8_t i_address, unsigned int numofbytes = 1){
	length_of_buffer = numofbytes;
	index = 0;
	if(numofbytes==1){
		/*Read a single byte */
		//The 'device address' is used to access slave, set Transfer Direction Bit to 1
		TWI1->TWI_MMR |= TWI_MMR_IADRSZ_1_BYTE | TWI_MMR_MREAD;
		//The Internal Address
		TWI1->TWI_IADR = TWI_IADR_IADR(i_address);
		//Read Single Byte
		TWI1->TWI_CR |= TWI_CR_START | TWI_CR_STOP;
		//Read Status register, wait until RXRDY is 1
		while(!(TWI1->TWI_SR & TWI_SR_RXRDY));
		//Read Receive Holding register
		buffer[0] = TWI1->TWI_RHR;
		//Read Status Register
		while(!(TWI1->TWI_SR & TWI_SR_TXCOMP));		
	}
	
	else {
		/*Read multiple  byte */
		//The 'device address' is used to access slave, set Transfer Direction Bit to 1
		TWI1->TWI_MMR |= TWI_MMR_IADRSZ_1_BYTE | TWI_MMR_MREAD;
		//The Internal Address
		TWI1->TWI_IADR = TWI_IADR_IADR(i_address);
		//START
		TWI1->TWI_CR |= TWI_CR_START;
		for(unsigned int i = 0; i<numofbytes-1; i++){
			//Read Status register, wait until RXRDY is 1
			while(!(TWI1->TWI_SR & TWI_SR_RXRDY));
			//Read Receive Holding register
			buffer[i] = TWI1->TWI_RHR;
		}
		//STOP
		TWI1->TWI_CR |= TWI_CR_STOP;
		//Read Status register, wait until RXRDY is 1
		while(!(TWI1->TWI_SR & TWI_SR_RXRDY));
		//Read Receive Holding register
		buffer[numofbytes-1] = TWI1->TWI_RHR;
		//Read Status Register
		while(!(TWI1->TWI_SR & TWI_SR_TXCOMP));
	}
	
}

void TWIrequestFrom2(uint8_t i_address_1, uint8_t i_address_2, unsigned int numofbytes = 1){
	length_of_buffer = numofbytes;
	index = 0;
	if(numofbytes==1){
		/*Read a single byte */
		//The 'device address' is used to access slave, set Transfer Direction Bit to 1
		TWI1->TWI_MMR |= TWI_MMR_IADRSZ_2_BYTE | TWI_MMR_MREAD;
		//The Internal Address
		TWI1->TWI_IADR = TWI_IADR_IADR(i_address_1<<8|i_address_2);
		//Read Single Byte
		TWI1->TWI_CR |= TWI_CR_START | TWI_CR_STOP;
		//Read Status register, wait until RXRDY is 1
		while(!(TWI1->TWI_SR & TWI_SR_RXRDY));
		//Read Receive Holding register
		buffer[0] = TWI1->TWI_RHR;
		//Read Status Register
		while(!(TWI1->TWI_SR & TWI_SR_TXCOMP));
	}
	
	else {
		/*Read multiple  byte */
		//The 'device address' is used to access slave, set Transfer Direction Bit to 1
		TWI1->TWI_MMR |= TWI_MMR_IADRSZ_2_BYTE | TWI_MMR_MREAD;
		//The Internal Address
		TWI1->TWI_IADR = TWI_IADR_IADR(i_address_1<<8|i_address_2);
		//START
		TWI1->TWI_CR |= TWI_CR_START;
		for(unsigned int i = 0; i<numofbytes-1; i++){
			//Read Status register, wait until RXRDY is 1
			while(!(TWI1->TWI_SR & TWI_SR_RXRDY));
			//Read Receive Holding register
			buffer[i] = TWI1->TWI_RHR;
		}
		//STOP
		TWI1->TWI_CR |= TWI_CR_STOP;
		//Read Status register, wait until RXRDY is 1
		while(!(TWI1->TWI_SR & TWI_SR_RXRDY));
		//Read Receive Holding register
		buffer[numofbytes-1] = TWI1->TWI_RHR;
		//Read Status Register
		while(!(TWI1->TWI_SR & TWI_SR_TXCOMP));
	}
	
}

#endif