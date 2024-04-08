/*
 * Kalman Filter.cpp
 *
 * Created: 01-07-2017 06:41:06
 * Author : Jyotirmaya Mahanta
 */ 


#include "sam.h"
#include "Kalman_IMU.h"
#include "InitTWI.h"
#include "InitUART.h"
#include "InitTimer.h"
#include <math.h>

void set_mpu_registers(void);
void read_mpu_data(void);
void calibrate_mpu(void);
void clear_variables(void);

int dt = 8000;
int temperature;
short acc_axis[4], gyro_axis[4];
long acc_x, acc_y, acc_z, acc_total_vector;
double gyro_pitch, gyro_roll, gyro_yaw;
int cal_int;
double gyro_axis_cal[4], acc_axis_cal[4];
float angle_pitch_acc, angle_roll_acc, angle_yaw_com;
float angle_pitch, angle_roll;

double angle_avg_roll, angle_avg_pitch, angle_avg_yaw, angle_var_roll, angle_var_pitch, angle_var_yaw;
double rate_avg_roll, rate_avg_pitch, rate_avg_yaw, rate_var_roll, rate_var_pitch, rate_var_yaw;

Kalman_IMU roll, pitch, yaw;

unsigned long loop_timer;

int main(void){
    SystemInit();
	TIMERbegin();
	UARTbegin(57600);
	TWIbegin();
	TWI_ConfigureMaster(TWI1, 400000UL, 84000000UL);
	
	writeln("Setting up MPU 6050 registers...");
	set_mpu_registers();
	write("Calibrating the gyroscope");
	calibrate_mpu();
	newline();
	writeln("Gyroscope calibrated.");
	
	read_mpu_data();
	
	roll.init_main(dt, angle_roll_acc, 0);
	pitch.init_main(dt, angle_pitch_acc, 0);
	yaw.init_main(dt, 0, 0);
	
	roll.init_processCov(0, 0);
	pitch.init_processCov(0, 0);
	yaw.init_processCov(0, 0);
	
	roll.init_measureCov(0, 0);
	pitch.init_measureCov(0, 0);
	yaw.init_measureCov(0, 0);
	
	loop_timer = micros();
    while (1) {
				
		clear_variables();
		int num = 4;
		for(int i=0; i<num; i++){
			
			read_mpu_data();
			
			//Accelerometer angle calculations
			acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));       //Calculate the total accelerometer vector.
			
			if(abs(acc_y) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
				angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;          //Calculate the pitch angle.
			}
			if(abs(acc_x) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
				angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;          //Calculate the roll angle.
			}
			
			angle_avg_roll += angle_roll_acc/num;
			angle_avg_pitch += angle_pitch_acc/num;
			angle_avg_yaw += angle_yaw_com/num;
			
			angle_var_roll += pow(angle_roll_acc, 2)/num;
			angle_var_pitch += pow(angle_pitch_acc, 2)/num;
			angle_var_yaw += pow(angle_yaw_com, 2)/num;
			
			rate_avg_roll += gyro_roll/num;
			rate_avg_pitch += gyro_pitch/num;
			rate_avg_yaw += gyro_yaw/num;
			
			rate_var_roll += pow(gyro_roll, 2)/num;
			rate_var_pitch += pow(gyro_pitch, 2)/num;
			rate_var_yaw += pow(gyro_yaw, 2)/num;
		}
		
		angle_var_roll -= pow(angle_avg_roll, 2);
		angle_var_pitch -= pow(angle_avg_pitch, 2);
		angle_var_yaw -= pow(angle_avg_yaw, 2);
		
		rate_var_roll -= pow(gyro_roll, 2);
		rate_var_pitch -= pow(gyro_pitch, 2);
		rate_var_yaw -= pow(gyro_yaw, 2);
		
		roll.kalmanOutput(angle_avg_roll, angle_var_roll, rate_avg_roll, rate_var_roll);
		pitch.kalmanOutput(angle_avg_pitch, angle_var_pitch, rate_avg_pitch, rate_var_pitch);
		yaw.kalmanOutput(angle_avg_yaw, angle_var_yaw, rate_avg_yaw, rate_var_yaw);
		
		angle_pitch = pitch.angle;                                    //Calculate the traveled pitch angle and add this to the angle_pitch variable.
		angle_roll = roll.angle;                                      //Calculate the traveled roll angle and add this to the angle_roll variable.
		
		//0.01745329251 = (3.142(PI) / 180degr) The Arduino sin function is in radians
		angle_pitch -= roll.angle * sin(yaw.angle * 0.01745329251);                  //If the IMU has yawed transfer the roll angle to the pitch angle.
		angle_roll += pitch.angle * sin(yaw.angle * 0.01745329251);                  //If the IMU has yawed transfer the pitch angle to the roll angle.
		
		
		println(roll.rate);
		
		//if(micros()-loop_timer > dt) writeln("Insufficient time");		
		while(micros() - loop_timer < dt);
		loop_timer = micros();
		
    }
}

void clear_variables(void){
	angle_avg_roll = 0;
	angle_avg_pitch = 0;
	angle_avg_yaw = 0;
	
	rate_avg_roll = 0;
	rate_avg_pitch = 0;
	rate_avg_yaw = 0;
	
	angle_var_roll = 0;
	angle_var_pitch = 0;
	angle_var_yaw = 0;
	
	rate_var_roll = 0;
	rate_var_pitch = 0;
	rate_var_yaw = 0;	
}

void calibrate_mpu(void){
	for(cal_int = 0; cal_int<2000; cal_int++){
		if(cal_int%125 == 0) write('.');
		read_mpu_data();
		gyro_axis_cal[1] += gyro_axis[1];
		gyro_axis_cal[2] += gyro_axis[2];
		gyro_axis_cal[3] += gyro_axis[3];
	}
	gyro_axis_cal[1] /= 2000;
	gyro_axis_cal[2] /= 2000;
	gyro_axis_cal[3] /= 2000;
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

	//We want to write to the CONFIG register (1A hex)
	//Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
	TWI_StartWrite(TWI1, mpu_address, 0x1A, 1, 0x03 );
	TWI_Stop(TWI1);
	TWI_WaitByteSent(TWI1, XMIT_TIMEOUT);
	TWI_WaitTransferComplete(TWI1, XMIT_TIMEOUT);
}
