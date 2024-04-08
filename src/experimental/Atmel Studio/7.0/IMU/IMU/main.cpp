/*
 * IMU.cpp
 *
 * Created: 04-07-2017 04:20:49
 * Author : Jyotirmaya Mahanta
 */ 


#include "sam3x8e.h"
#include <math.h>
#include "InitTimer.h"
#include "InitTWI.h"
#include "InitUART.h"
#include "Kalman_IMU.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void yaw_buffer_calculations(void);
void pitch_buffer_calculations(void);
void roll_buffer_calculations(void);
void calibrate_mpu(void);
void read_mpu_data(void);
void set_mpu_registers(void);
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

short temperature;
short acc_axis[4], gyro_axis[4];
double gyro_axis_cal[4];
int cal_int;
double gyro_pitch, gyro_roll, gyro_yaw;
long acc_x, acc_y, acc_z, acc_total_vector;
float angle_roll_acc, angle_pitch_acc, angle_yaw_com, angle_pitch, angle_roll;
float angle_roll_gyro, angle_pitch_gyro, angle_yaw_gyro;
bool gyro_angles_set;
uint32_t loop_timer;

uint8_t index = 0;
uint8_t num = 5;
#define buffer_length 5   
//average
double angle_avgbuffer_roll[buffer_length], angle_avgbuffer_pitch[buffer_length], angle_avgbuffer_yaw[buffer_length];
double rate_avgbuffer_roll[buffer_length], rate_avgbuffer_pitch[buffer_length], rate_avgbuffer_yaw[buffer_length];
//variance
double angle_devbuffer_roll[buffer_length], angle_devbuffer_pitch[buffer_length], angle_devbuffer_yaw[buffer_length];
double rate_devbuffer_roll[buffer_length], rate_devbuffer_pitch[buffer_length], rate_devbuffer_yaw[buffer_length];

double angle_avg_roll, angle_avg_pitch, angle_avg_yaw, angle_dev_roll, angle_dev_pitch, angle_dev_yaw;
double rate_avg_roll, rate_avg_pitch, rate_avg_yaw, rate_dev_roll, rate_dev_pitch, rate_dev_yaw;
double angle_var_roll, angle_var_pitch, angle_var_yaw, rate_var_roll, rate_var_pitch, rate_var_yaw;

#define dt 4000
Kalman_IMU roll, pitch, yaw;

int main(void){
	
    SystemInit();
	TIMERbegin();
	UARTbegin(57600);
	TWIbegin();
	//Set the I2C clock speed to 400kHz.
	TWI_ConfigureMaster(TWI1, 400000UL, 84000000UL);
	PIOB->PIO_OER |= PIO_OER_P27;
	writeln("setting up");
	set_mpu_registers();                                                     //Set the specific gyro registers.
	writeln("setup gyro done");
	
	write("calibrating");
	calibrate_mpu();
	writeln("calibrated...");
	
	loop_timer = micros();
	while(1){
		read_mpu_data();
						
		angle_pitch_gyro += gyro_pitch * 0.004;
		angle_roll_gyro += gyro_roll * 0.004;
		angle_yaw_gyro += gyro_yaw * 0.004;
		
		
		//Gyro angle calculations
		//0.004 = 1 / (250Hz)
		angle_pitch += gyro_pitch * 0.004;                                        //Calculate the traveled pitch angle and add this to the angle_pitch variable.
		angle_roll += gyro_roll * 0.004;                                          //Calculate the traveled roll angle and add this to the angle_roll variable.

		//0.00006981 = 0.004 * (3.142(PI) / 180degr) The Arduino sin function is in radians
		angle_pitch -= angle_roll * sin(gyro_yaw * 0.00006981);                  //If the IMU has yawed transfer the roll angle to the pitch angel.
		angle_roll += angle_pitch * sin(gyro_yaw * 0.00006981);                  //If the IMU has yawed transfer the pitch angle to the roll angel.

		//Accelerometer angle calculations
		acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));       //Calculate the total accelerometer vector.
		
		if(abs(acc_y) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
			angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;          //Calculate the pitch angle.
		}
		if(abs(acc_x) < acc_total_vector){                                          //Prevent the asin function to produce a NaN
			angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;          //Calculate the roll angle.
		}
		
		//Place the MPU-6050 spirit level and note the values in the following two lines for calibration.
		angle_pitch_acc -= 0.0;                                                   //Accelerometer calibration value for pitch.
		angle_roll_acc -= 0.0;                                                    //Accelerometer calibration value for roll.
		
		angle_pitch = angle_pitch * 0.996 + angle_pitch_acc * 0.004;            //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
		angle_roll = angle_roll * 0.996 + angle_roll_acc * 0.004;               //Correct the drift of the gyro roll angle with the accelerometer roll angle.
		
		//print(acc_axis[1]);
		//tabspace();
		print(angle_roll_acc);
		tabspace();
		//print(angle_pitch_gyro);
		//tabspace();
		println(angle_roll);
		while(micros() - loop_timer < 4000);
		loop_timer = micros();
	}
	
	read_mpu_data();       //for initialization below
	
	roll.init_main(dt, angle_roll_acc, 0);
	pitch.init_main(dt, angle_pitch_acc, 0);
	yaw.init_main(dt, 0, 0);
	
	roll.init_processCov(0.001, 0.0000001);
	pitch.init_processCov(0.001, 0.000000001);
	yaw.init_processCov(0.000001, 0.0000001);
	
	roll.init_measureCov(0, 0);
	pitch.init_measureCov(0, 0);
	yaw.init_measureCov(0, 0);
	
	loop_timer = micros();
	
	while(1){
		
		//Read gyro and accelerometer data
		read_mpu_data();
		
		//Read compass data
		
		//Compass angle calculations
		
		//Calculate variance and average
		yaw_buffer_calculations();
		
		
		yaw.kalmanOutput(angle_yaw_com, angle_var_yaw, gyro_yaw, rate_var_yaw);
				
		//Gyro roll pitch rate Calculations
		//0.01745329251 = (3.142(PI) / 180degr) The Arduino sin function is in radians
		double temp = gyro_pitch;
		gyro_pitch -=  (angle_roll_acc*250 + gyro_roll) * sin(yaw.angle * 0.01745329251);      //If the IMU has yawed transfer the roll angle to the pitch angle.
		gyro_roll += (angle_pitch_acc*250 + temp) * sin(yaw.angle * 0.01745329251);            //If the IMU has yawed transfer the pitch angle to the roll angle.
		
		angle_pitch_gyro += gyro_pitch * 0.004;
		angle_roll_gyro += gyro_roll * 0.004;
		angle_yaw_gyro += gyro_yaw * 0.004;

		//Accelerometer angle calculations
		acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));         //Calculate the total accelerometer vector.
		
		if(abs(acc_y) < acc_total_vector){                                          //Prevent the asin function to produce a NaN
			angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;          //Calculate the pitch angle.
		}
		if(abs(acc_x) < acc_total_vector){                                          //Prevent the asin function to produce a NaN
			angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;          //Calculate the roll angle.
		}
		
		//Calculate variance and average
		roll_buffer_calculations();
		pitch_buffer_calculations();
		
		roll.kalmanOutput(angle_roll_acc, angle_var_roll*100, gyro_roll, rate_var_roll*100);
		pitch.kalmanOutput(angle_pitch_acc, angle_var_pitch*100, gyro_pitch, rate_var_pitch*100);
		
		//Place the MPU-6050 spirit level and note the values in the following two lines for calibration.
		angle_pitch_acc -= 0.0;                                                   //Accelerometer calibration value for pitch.
		angle_roll_acc -= 0.0;                                                    //Accelerometer calibration value for roll.
		
		//angle_pitch = angle_pitch * 0.996 + angle_pitch_acc * 0.004;            //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
		//angle_roll = angle_roll * 0.996 + angle_roll_acc * 0.004;               //Correct the drift of the gyro roll angle with the accelerometer roll angle.
		
		//print(acc_axis[1]);
		//tabspace();
		print(angle_roll_acc);
		//tabspace();
		//print(angle_avg_roll);
		tabspace();
		println(roll.angle);
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
	
	gyro_roll = gyro_axis[1]/65.5;                      //Set gyro_roll to the correct axis that was stored in the EEPROM.
	//gyro_roll *= -1;                             //Invert gyro_roll if the MSB of EEPROM bit 28 is set.
	gyro_pitch = gyro_axis[2]/65.5;                     //Set gyro_pitch to the correct axis that was stored in the EEPROM.
	gyro_pitch *= -1;                              //Invert gyro_pitch if the MSB of EEPROM bit 29 is set.
	gyro_yaw = gyro_axis[3]/65.5;                       //Set gyro_yaw to the correct axis that was stored in the EEPROM.
	gyro_yaw *= -1;                                //Invert gyro_yaw if the MSB of EEPROM bit 30 is set.

	acc_x = acc_axis[2];                           //Set acc_x to the correct axis that was stored in the EEPROM.
	acc_x *= -1;                                   //Invert acc_x if the MSB of EEPROM bit 29 is set.
	acc_y = acc_axis[1];                           //Set acc_y to the correct axis that was stored in the EEPROM.
	//acc_y *= -1;                                 //Invert acc_y if the MSB of EEPROM bit 28 is set.
	acc_z = acc_axis[3];                           //Set acc_z to the correct axis that was stored in the EEPROM.
	acc_z *= -1;                                   //Invert acc_z if the MSB of EEPROM bit 30 is set.
	
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
		gyro_axis_cal[1] += gyro_axis[1];                                       //Add roll value to gyro_roll_cal.
		gyro_axis_cal[2] += gyro_axis[2];                                       //Add pitch value to gyro_pitch_cal.
		gyro_axis_cal[3] += gyro_axis[3];                                       //Add yaw value to gyro_yaw_cal.
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
	
	temp_avg = gyro_roll/num;
	temp_dev = pow(gyro_roll, 2)/num;
	
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
	
	temp_avg = gyro_pitch/num;
	temp_dev = pow(gyro_pitch, 2)/num;
	
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
	
	temp_avg = gyro_yaw/num;
	temp_dev = pow(gyro_yaw, 2)/num;
	
	rate_avg_yaw += temp_avg ;
	rate_avg_yaw -= rate_avgbuffer_yaw[index];						//rate yaw average
	rate_avgbuffer_yaw[index] = temp_avg;
	
	rate_dev_yaw += temp_dev;
	rate_dev_yaw -= rate_devbuffer_yaw[index];
	rate_devbuffer_yaw[index] = temp_dev;
	
	rate_var_yaw = rate_dev_yaw - pow(rate_avg_yaw, 2);				//rate yaw variance
	
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}