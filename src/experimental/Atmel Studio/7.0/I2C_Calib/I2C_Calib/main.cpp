/*
 * I2C_Calib.cpp
 *
 * Created: 03-07-2017 15:53:50
 * Author : Jyotirmaya Mahanta
 */ 


#include "sam3x8e.h"
#include <math.h>
#include "InitTimer.h"
#include "InitTWI.h"
#include "InitUART.h"
#include "InitADC.h"

//VCC---5V
//GND---GND
//SCL---SCL
//SDA---SDA
//PS ---5V          //activating I2C interface
//CSB---GND         //don't leave unconnected

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_alt = 1.3;               //Gain setting for the roll P-controller
float pid_i_gain_alt = 0.04;              //Gain setting for the roll I-controller
float pid_d_gain_alt = 18.0;              //Gain setting for the roll D-controller
int pid_max_alt = 400;                    //Maximum output of the PID-controller (+/-)

bool position_hold = false;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ms_calculations(void);
void ms_readADC(void);
void ms_reset(void);
void ms_readProm(void);
void display_data(void);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


uint32_t loop_timer;
uint16_t C1, C2, C3, C4, C5, C6;
uint32_t D1, D2;
int32_t dT, TEMP,P;
int64_t OFF, SENS;
double T2, OFF2, SENS2;

int main(void)
{
    SystemInit();
	TIMERbegin();
	UARTbegin(57600);
	TWIbegin();
	//Set the I2C clock speed to 400kHz.
	TWI_ConfigureMaster(TWI1, 400000UL, 84000000UL);
	ADCbegin();
	
	//reset the ms5611 once after power on
	ms_reset();
	//READ PROM DATA
	ms_readProm();                  //Read calibration data (factory calibrated) from PROM
	
	loop_timer = micros();                                                    //Set the timer for the next loop.

    while (1){
				
		//All the information for controlling the motor's is available.
		//The refresh rate is 250Hz. That means the ESC's need there pulse every 4ms.
		//while(micros() - loop_timer < 4000);                                      //We wait until 4000us are passed.
		//loop_timer = micros();                                                    //Set the timer for the next loop.
		
		ms_readADC();
		ms_calculations();
		display_data();
	}
}


void ms_readProm(void){
	
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//Subroutine for reading the barometer
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	//Read the MS5611
	uint8_t ms_address = 0x76;
	
	/*TWI_StartWrite(TWI1, ms_address, 0x6B, 1, 0x00 );
	TWI_Stop(TWI1);
	TWI_WaitByteSent(TWI1, XMIT_TIMEOUT);
	TWI_WaitTransferComplete(TWI1, XMIT_TIMEOUT);*/
	
	TWI_StartRead(TWI1, ms_address, 0xAE, 1);
	
	TWI_WaitByteReceived(TWI1, RECV_TIMEOUT);
	C1 = TWI_ReadByte(TWI1)<<8;
	TWI_WaitByteReceived(TWI1, RECV_TIMEOUT);
	C1 |= TWI_ReadByte(TWI1);
	
	TWI_WaitByteReceived(TWI1, RECV_TIMEOUT);
	C2 = TWI_ReadByte(TWI1)<<8;
	TWI_WaitByteReceived(TWI1, RECV_TIMEOUT);
	C2 |= TWI_ReadByte(TWI1);
	
	TWI_WaitByteReceived(TWI1, RECV_TIMEOUT);
	C3 = TWI_ReadByte(TWI1)<<8;
	TWI_WaitByteReceived(TWI1, RECV_TIMEOUT);
	C3 |= TWI_ReadByte(TWI1);
	
	TWI_WaitByteReceived(TWI1, RECV_TIMEOUT);
	C4 = TWI_ReadByte(TWI1)<<8;
	TWI_WaitByteReceived(TWI1, RECV_TIMEOUT);
	C4 |= TWI_ReadByte(TWI1);
	
	TWI_WaitByteReceived(TWI1, RECV_TIMEOUT);
	C5 = TWI_ReadByte(TWI1)<<8;
	TWI_WaitByteReceived(TWI1, RECV_TIMEOUT);
	C5 |= TWI_ReadByte(TWI1);
	
	TWI_WaitByteReceived(TWI1, RECV_TIMEOUT);
	C6 = TWI_ReadByte(TWI1)<<8;
	TWI_Stop(TWI1);
	TWI_WaitByteReceived(TWI1, RECV_TIMEOUT);
	C6 |= TWI_ReadByte(TWI1);
	TWI_WaitTransferComplete(TWI1, RECV_TIMEOUT);
	
}

void ms_readADC(void){
	//Setup the MPU-6050
	uint8_t ms_address = 0x76;
		
	//convert D1 ; OSR = 4096
	//conversion takes 8.22ms
	TWI_StartWrite(TWI1, ms_address, 0, 0, 0x48 );
	TWI_Stop(TWI1);
	TWI_WaitByteSent(TWI1, XMIT_TIMEOUT);	
	TWI_WaitTransferComplete(TWI1, XMIT_TIMEOUT);
	
	_delay_ms(9);
	//read pressure	
	TWI_StartRead(TWI1, ms_address, 0x00, 1);
	TWI_WaitByteReceived(TWI1, RECV_TIMEOUT);
	D1 = TWI_ReadByte(TWI1)<<16;
	TWI_WaitByteReceived(TWI1, RECV_TIMEOUT);
	D1 |= TWI_ReadByte(TWI1)<<8;
	TWI_WaitByteReceived(TWI1, RECV_TIMEOUT);
	D1 |= TWI_ReadByte(TWI1);
    
	//convert D2 ; OSR = 4096
	//conversion takes 8.22ms
	TWI_StartWrite(TWI1, ms_address, 0, 0, 0x58 );
	TWI_Stop(TWI1);
	TWI_WaitByteSent(TWI1, XMIT_TIMEOUT);
	TWI_WaitTransferComplete(TWI1, XMIT_TIMEOUT);
	
	_delay_ms(9);
	//read pressure
	TWI_StartRead(TWI1, ms_address, 0x00, 1);
	TWI_WaitByteReceived(TWI1, RECV_TIMEOUT);
	D2 = TWI_ReadByte(TWI1)<<16;
	TWI_WaitByteReceived(TWI1, RECV_TIMEOUT);
	D2 |= TWI_ReadByte(TWI1)<<8;
	TWI_WaitByteReceived(TWI1, RECV_TIMEOUT);
	D2 |= TWI_ReadByte(TWI1);
}

void ms_reset(void){
	uint8_t ms_address = 0x76;
	
	TWI_StartWrite(TWI1, ms_address, 0, 0, 0x1E );
	TWI_Stop(TWI1);
	TWI_WaitByteSent(TWI1, XMIT_TIMEOUT);
	TWI_WaitTransferComplete(TWI1, XMIT_TIMEOUT);
}

void ms_calculations(void){
	ms_readADC();                                                      //Read digital pressure and temperature data
	dT = D2 - C5*pow(2,8);                                          //Calculate Temperature
	TEMP = 2000 + dT*C6/pow(2,23);
	OFF = C2*pow(2,16) + C4*dT/pow(2,7);
	SENS = C1*pow(2,15) + C3*dT/pow(2,8);
	////////// SECOND ORDER TEMPERATURE COMPENSATION ///////////
	if(TEMP<2000)
	{
		T2 = pow(dT,2)/pow(2,31);
		OFF2 = 2.5*pow((TEMP-2000),2);
		SENS2 = 1.25*pow((TEMP-2000),2);
		TEMP -= T2;
		OFF -= OFF2;
		SENS -= SENS2;
	}
	////////////////////////////////////////////////////////////
	P = (D1*SENS/pow(2,21) - OFF)/pow(2,15);
}

void display_data(void){
	//Display data on screen
	write("PRESSURE : ");
	print(P/100.0);
	write("          TEMPERATURE : ");
	print(TEMP);
	newline();
}