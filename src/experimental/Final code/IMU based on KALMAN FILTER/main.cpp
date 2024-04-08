/*
 * IMU_2.cpp
 *
 * Created: 07-07-2017 18:29:26
 * Author : Jyotirmaya Mahanta
 */ 



#include "sam3x8e.h"
#include <math.h>
#include "InitTimer.h"
#include "InitTWI.h"
#include "InitUART.h"
#include "Kalman_IMU.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//KALMAN FILTER PROCESS NOISE COVARIANCES
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double processCov_roll = 0.0000003;
double processCov_pitch = processCov_roll;
double processCov_yaw = 0;
//variables for Kalman Filtering
#define dt 4000
Kalman_IMU roll, pitch, yaw;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void yaw_buffer_calculations(void);
void pitch_buffer_calculations(void);
void roll_buffer_calculations(void);
void calibrate_mpu(void);
void read_mpu_data(void);
void read_com_data(void);
void set_mpu_registers(void);
//void set_com_registers(void);
void relative_to_euler(double w_x, double w_y, double w_z);
void displayData(void);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

short temperature;
short acc_axis[4], gyro_axis[4];
double gyro_axis_cal[4];
int cal_int;
double gyro_y, gyro_x, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector;
double angle_roll_acc, angle_pitch_acc, angle_yaw_com;
double rate_roll, rate_pitch, rate_yaw;
uint32_t loop_timer;

/////////////////////////////////variables for variance calculation
uint8_t index = 0;
uint8_t num = 5;
#define buffer_length 5
//average buffer
double angle_avgbuffer_roll[buffer_length], angle_avgbuffer_pitch[buffer_length], angle_avgbuffer_yaw[buffer_length];
double rate_avgbuffer_roll[buffer_length], rate_avgbuffer_pitch[buffer_length], rate_avgbuffer_yaw[buffer_length];
//variance buffer
double angle_devbuffer_roll[buffer_length], angle_devbuffer_pitch[buffer_length], angle_devbuffer_yaw[buffer_length];
double rate_devbuffer_roll[buffer_length], rate_devbuffer_pitch[buffer_length], rate_devbuffer_yaw[buffer_length];

//average and deviation
double angle_avg_roll, angle_avg_pitch, angle_avg_yaw, angle_dev_roll, angle_dev_pitch, angle_dev_yaw;
double rate_avg_roll, rate_avg_pitch, rate_avg_yaw, rate_dev_roll, rate_dev_pitch, rate_dev_yaw;
//variance
double angle_var_roll, angle_var_pitch, angle_var_yaw, rate_var_roll, rate_var_pitch, rate_var_yaw;


int main(void){
	
	SystemInit();
	TIMERbegin();
	UARTbegin(57600);
	TWIbegin();
	//Set the I2C clock speed to 400kHz.
	TWI_ConfigureMaster(TWI1, 400000UL, 84000000UL);
	PIOB->PIO_OER |= PIO_OER_P27;
	//set_com_registers();
	writeln("setting up");
	set_mpu_registers();                                                     //Set the specific gyro registers.
	writeln("setup gyro done");
	
	write("calibrating");
	calibrate_mpu();
	writeln("calibrated...");
	
	read_mpu_data();       //for initialization below
	
	//initialize everything
	roll.init_main(dt, angle_roll_acc);
	pitch.init_main(dt, angle_pitch_acc);
	yaw.init_main(dt, 0);
	
	roll.init_processCov(processCov_roll);
	pitch.init_processCov(processCov_pitch);
	yaw.init_processCov(processCov_yaw);
	
	roll.init_measureCov(0);
	pitch.init_measureCov(0);
	yaw.init_measureCov(0);
	
	loop_timer = micros();
	
////////////////////////////////////////////////////////////////KALMAN FILTER SINGLE STATE VARIABLE///////////////////////////////////////////////////////////////////////////////////
	while(1){
		
		//Read gyro and accelerometer data
		read_mpu_data();
		
		//Read compass data
		//read_com_data();
				
		//Accelerometer Euler angle calculations
		acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));         //Calculate the total accelerometer vector.
		
		if(abs(acc_y) < acc_total_vector){                                          //Prevent the asin function to produce a NaN
			angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;          //Calculate the pitch angle.
		}
		if(abs(acc_x) < acc_total_vector){                                          //Prevent the asin function to produce a NaN
			angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;          //Calculate the roll angle.
		}
		
		//Accelerometer offset calibration values.
		angle_pitch_acc -= 0.0;                                                   //Accelerometer calibration value for pitch.
		angle_roll_acc -= 0.0;                                                    //Accelerometer calibration value for roll.
		
		//Gyro Euler rate Calculations		
		relative_to_euler(gyro_x, gyro_y, gyro_z);      //updates rate_roll, rate_pitch & rate_yaw variables
		
		//Compass (Euler/body) angle calculations
		
		//Calculate variance
		//yaw_buffer_calculations();
		roll_buffer_calculations();
		pitch_buffer_calculations();
		
		roll.kalmanOutput(angle_avg_roll, rate_roll, angle_var_roll, rate_var_roll);
		pitch.kalmanOutput(angle_avg_pitch, rate_pitch, angle_var_roll, rate_var_pitch);
		
		//roll.kalmanOutput(angle_roll_acc, rate_roll, angle_var_roll, rate_var_roll);
		//pitch.kalmanOutput(angle_pitch_acc, rate_pitch, angle_var_roll, rate_var_pitch);
		//yaw.kalmanOutput(angle_yaw_com, rate_yaw, angle_var_yaw, rate_var_yaw);
		
		displayData();
		
		while(micros() - loop_timer < 4000);
		loop_timer = micros();
	}
}

void read_mpu_data(void){
	//Read the MPU-6050
	uint8_t mpu_address = 0x68;
	
	//Start reading @ register 3Bh and auto increment with every read.
	TWI_StartRead(TWI1, mpu_address, 0x3B, 1);
	
	TWI_WaitByteReceived(TWI1, RECV_TIMEOUT);
	acc_axis[1] = TWI_ReadByte(TWI1)<<8;
	TWI_WaitByteReceived(TWI1, RECV_TIMEOUT);
	acc_axis[1] |= TWI_ReadByte(TWI1);
	
	TWI_WaitByteReceived(TWI1, RECV_TIMEOUT);
	acc_axis[2] = TWI_ReadByte(TWI1)<<8;
	TWI_WaitByteReceived(TWI1, RECV_TIMEOUT);
	acc_axis[2] |= TWI_ReadByte(TWI1);
	
	TWI_WaitByteReceived(TWI1, RECV_TIMEOUT);
	acc_axis[3] = TWI_ReadByte(TWI1)<<8;
	TWI_WaitByteReceived(TWI1, RECV_TIMEOUT);
	acc_axis[3] |= TWI_ReadByte(TWI1);
	
	TWI_WaitByteReceived(TWI1, RECV_TIMEOUT);
	temperature = TWI_ReadByte(TWI1)<<8;
	TWI_WaitByteReceived(TWI1, RECV_TIMEOUT);
	temperature |= TWI_ReadByte(TWI1);
	
	TWI_WaitByteReceived(TWI1, RECV_TIMEOUT);
	gyro_axis[1] = TWI_ReadByte(TWI1)<<8;
	TWI_WaitByteReceived(TWI1, RECV_TIMEOUT);
	gyro_axis[1] |= TWI_ReadByte(TWI1);
	
	TWI_WaitByteReceived(TWI1, RECV_TIMEOUT);
	gyro_axis[2] = TWI_ReadByte(TWI1)<<8;
	TWI_WaitByteReceived(TWI1, RECV_TIMEOUT);
	gyro_axis[2] |= TWI_ReadByte(TWI1);
	
	TWI_WaitByteReceived(TWI1, RECV_TIMEOUT);
	gyro_axis[3] = TWI_ReadByte(TWI1)<<8;
	TWI_Stop(TWI1);
	TWI_WaitByteReceived(TWI1, RECV_TIMEOUT);
	gyro_axis[3] |= TWI_ReadByte(TWI1);
	TWI_WaitTransferComplete(TWI1, RECV_TIMEOUT);
	
	
	acc_axis[1] -= 0.0;
	acc_axis[2] -= 0.0;
	acc_axis[3] -= 0.0;
	
	if(cal_int == 2000){
		gyro_axis[1] -= gyro_axis_cal[1];                                       //Only compensate after the calibration.
		gyro_axis[2] -= gyro_axis_cal[2];                                       //Only compensate after the calibration.
		gyro_axis[3] -= gyro_axis_cal[3];                                       //Only compensate after the calibration.
	}
	
	gyro_x = gyro_axis[1]/65.5;                      //Set gyro_x to the correct axis that was stored in the EEPROM.
	//gyro_x *= -1;                                  //Invert gyro_x 
	gyro_y = gyro_axis[2]/65.5;                      //Set gyro_y to the correct axis that was stored in the EEPROM.
	gyro_y *= -1;                                    //Invert gyro_y 
	gyro_z = gyro_axis[3]/65.5;                      //Set gyro_z to the correct axis that was stored in the EEPROM.
	gyro_z *= -1;                                    //Invert gyro_z 

	acc_x = acc_axis[2];                             //Set acc_x to the correct axis that was stored in the EEPROM.
	acc_x *= -1;                                     //Invert acc_x 
	acc_y = acc_axis[1];                             //Set acc_y to the correct axis that was stored in the EEPROM.
	//acc_y *= -1;                                   //Invert acc_y 
	acc_z = acc_axis[3];                             //Set acc_z to the correct axis that was stored in the EEPROM.
	acc_z *= -1;                                     //Invert acc_z 
	
}

void set_mpu_registers(void){
	//Setup the MPU-6050
	uint8_t mpu_address = 0x68;
	
	//We want to write to the PWR_MGMT_1 register (6B hex)
	//Set the register bits as 00000000 to activate the gyro
	TWI_StartWrite(TWI1, mpu_address, 0x6B, 1, 0x00 );
	TWI_Stop(TWI1);
	TWI_WaitByteSent(TWI1, XMIT_TIMEOUT);
	TWI_WaitTransferComplete(TWI1, XMIT_TIMEOUT);

	//We want to write to the GYRO_CONFIG register (1B hex)
	//Set the register bits as 00001000 (500dps full scale)
	TWI_StartWrite(TWI1, mpu_address, 0x1B, 1, 0x08 );
	TWI_Stop(TWI1);
	TWI_WaitByteSent(TWI1, XMIT_TIMEOUT);
	TWI_WaitTransferComplete(TWI1, XMIT_TIMEOUT);

	//We want to write to the ACCEL_CONFIG register (1A hex)
	//Set the register bits as 00010000 (+/- 8g full scale range)
	TWI_StartWrite(TWI1, mpu_address, 0x1C, 1, 0x10 );
	TWI_Stop(TWI1);
	TWI_WaitByteSent(TWI1, XMIT_TIMEOUT);
	TWI_WaitTransferComplete(TWI1, XMIT_TIMEOUT);

	//Let's perform a random register check to see if the values are written correct
	//Start reading @ register 0x1B
	TWI_StartRead(TWI1, mpu_address, 0x1B, 1);
	TWI_Stop(TWI1);
	TWI_WaitByteReceived(TWI1, RECV_TIMEOUT);
	uint8_t data = TWI_ReadByte(TWI1);
	TWI_WaitTransferComplete(TWI1, RECV_TIMEOUT);
	if(data != 0x08){
		//turn on warning led
		PIOB->PIO_SODR |= PIO_SODR_P27;
		while(1) _delay_ms(10);
	}

	//We want to write to the CONFIG register (1A hex)
	//Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
	TWI_StartWrite(TWI1, mpu_address, 0x1A, 1, 0x03 );
	TWI_Stop(TWI1);
	TWI_WaitByteSent(TWI1, XMIT_TIMEOUT);
	TWI_WaitTransferComplete(TWI1, XMIT_TIMEOUT);
}

void calibrate_mpu(void){
	//Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
	for (cal_int = 0; cal_int < 2000 ; cal_int ++){                           //Take 2000 readings for calibration.
		if(cal_int % 15 == 0){
			//Change the led status to indicate calibration.
			if((PIOB->PIO_ODSR & PIO_ODSR_P27) == PIO_ODSR_P27) PIOB->PIO_CODR |= PIO_CODR_P27;
			else PIOB->PIO_SODR |= PIO_SODR_P27;
		}
		read_mpu_data();                                                        //Read the gyro output.
		//println(gyro_axis[1]);
		gyro_axis_cal[1] += gyro_axis[1];                                       //Add roll value to gyro_x_cal.
		gyro_axis_cal[2] += gyro_axis[2];                                       //Add pitch value to gyro_y_cal.
		gyro_axis_cal[3] += gyro_axis[3];                                       //Add yaw value to gyro_z_cal.
		_delay_ms(2);                                                     //Wait 3000us.
	}
	//Now that we have 2000 measures, we need to divide by 2000 to get the average gyro offset.
	gyro_axis_cal[1] /= 2000;                                                 //Divide the roll total by 2000.
	gyro_axis_cal[2] /= 2000;                                                 //Divide the pitch total by 2000.
	gyro_axis_cal[3] /= 2000;                                                 //Divide the yaw total by 2000.
}

void roll_buffer_calculations(void){
	index++;
	if(index == buffer_length) index = 0;
	
	double temp_avg, temp_dev;
	
	//Roll calculations//////////////////////////////////////////////////////////////////////////////////////////////////
	temp_avg = angle_roll_acc/num;
	temp_dev = pow(angle_roll_acc, 2)/num;
	
	angle_avg_roll += temp_avg ;
	angle_avg_roll -= angle_avgbuffer_roll[index];                      //angle roll average
	angle_avgbuffer_roll[index] = temp_avg;
	
	angle_dev_roll += temp_dev;
	angle_dev_roll -= angle_devbuffer_roll[index];
	angle_devbuffer_roll[index] = temp_dev;
	
	angle_var_roll = angle_dev_roll - pow(angle_avg_roll, 2);			//angle roll variance
	
	temp_avg = rate_roll/num;
	temp_dev = pow(rate_roll, 2)/num;
	
	rate_avg_roll += temp_avg ;
	rate_avg_roll -= rate_avgbuffer_roll[index];						//rate roll average
	rate_avgbuffer_roll[index] = temp_avg;
	
	rate_dev_roll += temp_dev;
	rate_dev_roll -= rate_devbuffer_roll[index];
	rate_devbuffer_roll[index] = temp_dev;
	
	rate_var_roll = rate_dev_roll - pow(rate_avg_roll, 2);				//rate roll variance
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}

void pitch_buffer_calculations(void){
	index++;
	if(index == buffer_length) index = 0;
	
	double temp_avg, temp_dev;
	
	//Pitch calculations//////////////////////////////////////////////////////////////////////////////////////////////////
	temp_avg = angle_pitch_acc/num;
	temp_dev = pow(angle_pitch_acc, 2)/num;
	
	angle_avg_pitch += temp_avg ;
	angle_avg_pitch -= angle_avgbuffer_pitch[index];                      //angle pitch average
	angle_avgbuffer_pitch[index] = temp_avg;
	
	angle_dev_pitch += temp_dev;
	angle_dev_pitch -= angle_devbuffer_pitch[index];
	angle_devbuffer_pitch[index] = temp_dev;
	
	angle_var_pitch = angle_dev_pitch - pow(angle_avg_pitch, 2);			//angle pitch variance
	
	temp_avg = rate_pitch/num;
	temp_dev = pow(rate_pitch, 2)/num;
	
	rate_avg_pitch += temp_avg ;
	rate_avg_pitch -= rate_avgbuffer_pitch[index];						//rate pitch average
	rate_avgbuffer_pitch[index] = temp_avg;
	
	rate_dev_pitch += temp_dev;
	rate_dev_pitch -= rate_devbuffer_pitch[index];
	rate_devbuffer_pitch[index] = temp_dev;
	
	rate_var_pitch = rate_dev_pitch - pow(rate_avg_pitch, 2);				//rate pitch variance
	
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}

void yaw_buffer_calculations(void){
	index++;
	if(index == buffer_length) index = 0;
	
	double temp_avg, temp_dev;
	
	//yaw calculations//////////////////////////////////////////////////////////////////////////////////////////////////
	temp_avg = angle_yaw_com/num;
	temp_dev = pow(angle_yaw_com, 2)/num;
	
	angle_avg_yaw += temp_avg ;
	angle_avg_yaw -= angle_avgbuffer_yaw[index];                      //angle yaw average
	angle_avgbuffer_yaw[index] = temp_avg;
	
	angle_dev_yaw += temp_dev;
	angle_dev_yaw -= angle_devbuffer_yaw[index];
	angle_devbuffer_yaw[index] = temp_dev;
	
	angle_var_yaw = angle_dev_yaw - pow(angle_avg_yaw, 2);			//angle yaw variance
	
	temp_avg = rate_yaw/num;
	temp_dev = pow(rate_yaw, 2)/num;
	
	rate_avg_yaw += temp_avg ;
	rate_avg_yaw -= rate_avgbuffer_yaw[index];						//rate yaw average
	rate_avgbuffer_yaw[index] = temp_avg;
	
	rate_dev_yaw += temp_dev;
	rate_dev_yaw -= rate_devbuffer_yaw[index];
	rate_devbuffer_yaw[index] = temp_dev;
	
	rate_var_yaw = rate_dev_yaw - pow(rate_avg_yaw, 2);				//rate yaw variance
	
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}

void relative_to_euler(double w_x, double w_y, double w_z){
	// w refers to omega (angular rate about body axes)
	//0.01745329251 = (3.142(PI) / 180degr) The Arduino sin function is in radians
	rate_roll  = w_x + w_y*sin(roll.angle*0.01745329251)*tan(pitch.angle*0.01745329251) + w_z*cos(roll.angle*0.01745329251)*tan(pitch.angle*0.01745329251);
	rate_pitch =       w_y*cos(pitch.angle*0.01745329251)                               - w_z*sin(roll.angle*0.01745329251);
	rate_yaw   =       w_y*sin(roll.angle*0.01745329251)/cos(pitch.angle*0.01745329251) + w_z*cos(roll.angle*0.01745329251)*cos(pitch.angle*0.01745329251);
}

void displayData(void){
	print(angle_roll_acc);
	tabspace();
	//print();
	//tabspace();
	//print(angle_avg_roll);
	//tabspace();
	println(roll.angle);
}