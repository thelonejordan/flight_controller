/*
 * MPU6050.cpp
 *
 * Created: 24-06-2017 20:44:51
 * Author : Jyotirmaya Mahanta
 */ 

#include <math.h>
#include "sam3x8e.h"
#include "InitTimer.h"
#include "InitUART.h"
#include "InitTWI.h"

#define loop_time 10

void calibrate(void);
void read_data(void);
void setup_reg(void);

short gyro_x, gyro_y, gyro_z;                    //gyroscope output in deg/sec
int temperature;
double gyro_x_cal, gyro_y_cal, gyro_z_cal;          //gyroscope calibration values
int acc_x, acc_y, acc_z;                         //accelerometer raw output
double total_vector;
double angle_x, angle_y;                          //initial angle values
double acc_angle_x, acc_angle_y;                  //accelerometer output in deg
int cal_int;
bool set_angles = false;
long timer;

int main(void)
{
    SystemInit();
    TIMERbegin();
	TWIbegin();
	TWI_ConfigureMaster(TWI1, 100000UL, 84000000UL);
	UARTbegin(57600);
	
	
	UARTprintln("Setting up gyro registers...");
	setup_reg();
	UARTprint("Calibrating the gyro ");
	calibrate();
	UARTprintln("Calibrated ");
	timer = millis();
	
    while (1) {
		//while((millis()-timer)<loop_time);
		//timer = millis();
		
		read_data();
		UARTprintln(gyro_z);
		/*UARTprint("     ");
		UARTprint(gyro_y);
		UARTprint("     ");
	    UARTprintln(gyro_z);*/
		_delay_ms(10);
    }
}

void calibrate()
{
	for(cal_int=0; cal_int<2000; cal_int++)
	{
		if(cal_int%125==0)UARTprint(".");
		read_data();
		gyro_x_cal+=gyro_x;
		gyro_y_cal+=gyro_y;
		gyro_z_cal+=gyro_z;
	}
	gyro_x_cal/=2000;
	gyro_y_cal/=2000;
	gyro_z_cal/=2000;
	_delay_ms(5);
}


void read_data(void)
{
	
	TWI_StartRead(TWI1, 0x68, 0x3B, 1);
	
	if (TWI_WaitByteReceived(TWI1, RECV_TIMEOUT)) gyro_x  = TWI_ReadByte(TWI1)<<8;
	if (TWI_WaitByteReceived(TWI1, RECV_TIMEOUT)) gyro_x |= TWI_ReadByte(TWI1);
	
	if (TWI_WaitByteReceived(TWI1, RECV_TIMEOUT)) gyro_y  = TWI_ReadByte(TWI1)<<8;
	if (TWI_WaitByteReceived(TWI1, RECV_TIMEOUT)) gyro_y |= TWI_ReadByte(TWI1);
	
	if (TWI_WaitByteReceived(TWI1, RECV_TIMEOUT)) gyro_z  = TWI_ReadByte(TWI1)<<8;
	TWI_Stop(TWI1);
	if (TWI_WaitByteReceived(TWI1, RECV_TIMEOUT)) gyro_z |= TWI_ReadByte(TWI1);
	
	TWI_WaitTransferComplete(TWI1, RECV_TIMEOUT);
	
	if(cal_int==2000)
	{
		gyro_x=(gyro_x-gyro_x_cal);//14.375;
		gyro_y=(gyro_y-gyro_y_cal);//14.375;
		gyro_z=(gyro_z-gyro_z_cal);//14.375;
	}
	
}

/*void setup_reg(void){
	
    TWI_StartWrite(TWI1, 0x53, 0, 0, 0x2D);
	TWI_WaitByteSent(TWI1, XMIT_TIMEOUT);
	TWI_WriteByte(TWI1, 0x08);
	TWI_Stop(TWI1);
	TWI_WaitByteSent(TWI1, XMIT_TIMEOUT);
	TWI_WaitTransferComplete(TWI1, XMIT_TIMEOUT);
	
	TWI_StartWrite(TWI1, 0x53, 0, 0, 0x2C);
	TWI_WaitByteSent(TWI1, XMIT_TIMEOUT);
	TWI_WriteByte(TWI1, 0x0C);
	TWI_Stop(TWI1);
	TWI_WaitByteSent(TWI1, XMIT_TIMEOUT);
	TWI_WaitTransferComplete(TWI1, XMIT_TIMEOUT);
	
	TWI_StartWrite(TWI1, 0x53, 0, 0, 0x31);
	TWI_WaitByteSent(TWI1, XMIT_TIMEOUT);
	TWI_WriteByte(TWI1, 0x0F);
	TWI_Stop(TWI1);
	TWI_WaitByteSent(TWI1, XMIT_TIMEOUT);
	
	TWI_WaitTransferComplete(TWI1, XMIT_TIMEOUT);                           
}*/

void setup_reg(void){
	
	TWI_StartWrite(TWI1, 0x68, 0x6B, 1, 0x00 );
	TWI_Stop(TWI1);
	TWI_WaitByteSent(TWI1, XMIT_TIMEOUT);	
	TWI_WaitTransferComplete(TWI1, XMIT_TIMEOUT);
	
	TWI_StartWrite(TWI1, 0x68, 0x1C, 1, 0x10);
	TWI_Stop(TWI1);
	TWI_WaitByteSent(TWI1, XMIT_TIMEOUT);
	TWI_WaitTransferComplete(TWI1, XMIT_TIMEOUT);
	
	TWI_StartWrite(TWI1, 0x68, 0x1B, 1, 0x08 );
	TWI_Stop(TWI1);
	TWI_WaitByteSent(TWI1, XMIT_TIMEOUT);
	TWI_WaitTransferComplete(TWI1, XMIT_TIMEOUT);
}
