/*
 * i2c_master.cpp
 *
 * Created: 05-07-2017 17:23:43
 * Author : Jyotirmaya Mahanta
 */ 


#include "sam3x8e.h"
#include "InitTimer.h"
#include "InitUART.h"
#include "InitTWI.h"

void setup_interrupts(void);

int16_t data;
volatile uint8_t data_ready = 0;
volatile uint8_t nano_online = 0;
uint32_t pin_status;
uint32_t ledPin = (1u<<27);
uint32_t int_pin_tx = (1u<<28);   //D3 - 28
uint32_t int_pin_rx = (1u<<25);   //D2 - 25

int main(void){
    /* Initialize the SAM system */
    SystemInit();
	TIMERbegin();
	UARTbegin(57600);
	TWIbegin();
	TWI_ConfigureMaster(TWI1, 400000UL, 84000000UL);
	PIOC->PIO_PER |= int_pin_tx;
	PIOC->PIO_OER |= int_pin_tx;
	PIOB->PIO_PER |= PIO_PER_P27;
	PIOB->PIO_OER |= PIO_OER_P27;
	setup_interrupts();
	writeln("Starting");
	while(!nano_online) writeln("Nano offline");             //wait until nano is on line
	writeln("nano is on line");
    while (1){
		// toggle pin to notify slave to start reading data from mpu
		pin_status = PIOC->PIO_ODSR & int_pin_tx;
		if(pin_status) PIOC->PIO_CODR |= int_pin_tx;
		else PIOC->PIO_SODR |= int_pin_tx;
		
		_delay_ms(3);   //wait for data to be read
		
		//start reading data from slave
		while(!data_ready) writeln("DRDY");
		TWI_StartRead(TWI1, 8, 0, 0);
		TWI_WaitByteReceived(TWI1, RECV_TIMEOUT) ;
		data  = TWI_ReadByte(TWI1)<<8;
		TWI_Stop(TWI1);
		TWI_WaitByteReceived(TWI1, RECV_TIMEOUT);
		data |= TWI_ReadByte(TWI1);
		TWI_WaitTransferComplete(TWI1, RECV_TIMEOUT);
		println(data);
		data_ready = 0;
		
    }
	while (0){
		pin_status = PIOC->PIO_ODSR & int_pin_tx;
		if(pin_status) PIOC->PIO_CODR |= int_pin_tx;
		else PIOC->PIO_SODR |= int_pin_tx;
		_delay_ms(300);
	}
	while(0){
		pin_status = PIOB->PIO_ODSR & ledPin;
		if(pin_status) PIOB->PIO_CODR |= ledPin;
		else PIOB->PIO_SODR |= ledPin;
		_delay_ms(300);
	}
}

void PIOB_Handler(){
	if(!nano_online){
		// notify : i am on line
		pin_status = PIOC->PIO_ODSR & int_pin_tx;
		if(pin_status) PIOC->PIO_CODR |= int_pin_tx;
		else PIOC->PIO_SODR |= int_pin_tx;
		nano_online = 1;     //check for first toggle
	}
	else data_ready = 1;
	
	PIOB->PIO_ISR;
}

void setup_interrupts(void){
	PMC->PMC_PCER0 |= (1u<<ID_PIOB);        //enable peripheral clock
	PIOB->PIO_IER |= int_pin_rx;               //enable pin change interrupts
	PIOB->PIO_PER |= int_pin_rx;               //enable PIO controller
	PIOB->PIO_ODR |= int_pin_rx;               //set as input
	NVIC_SetPriority(PIOB_IRQn, 0);			//set priority
	NVIC_EnableIRQ(PIOB_IRQn);              //configure NVIC
}