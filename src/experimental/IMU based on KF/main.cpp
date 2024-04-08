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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
uint32_t loop_timer;


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
	
	/////////////////////////////////////////////////COMPLIMENTARY FILTER PART 1///////////////////////////////////////////////////////////////////////////////////////////
	loop_timer = micros();
	while(0){
		read_mpu_data();
		
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
		print(angle_pitch_acc);
		tabspace();
		//print(angle_pitch_gyro);
		//tabspace();
		println(angle_pitch);
		while(micros() - loop_timer < 4000);
		loop_timer = micros();
	}
	
/////////////////////////////////////////////////COMPLIMENTARY FILTER PART 2///////////////////////////////////////////////////////////////////////////////////////////
	loop_timer = micros();
	while(1){
		read_mpu_data();
		
		
		//Gyro angle calculations
		//0.004 = 1 / (250Hz)
		double temp_angle_roll = angle_roll;
		angle_roll += 0.004 * (gyro_roll + sin(angle_roll * 0.00006981)*tan(angle_pitch * 0.00006981)*gyro_pitch + cos(angle_roll * 0.00006981)*tan(angle_pitch * 0.00006981)*gyro_yaw);
		angle_pitch += 0.004 * (cos(angle_pitch * 0.00006981)*gyro_pitch - sin(temp_angle_roll * 0.00006981)*gyro_yaw);

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


