/*
 * Flight Controller (KF + PWM).cpp
 *
 * Created: 14-07-2017 16:28:02
 * Author : Jyotirmaya Mahanta
 */ 

#include <math.h>
#include "sam3x8e.h"
#include "InitTimer.h"
#include "InitTWI.h"
#include "InitUART.h"
#include "InitADC.h"
#include "InitPWM.h"

#include "VAR.h"
#include "Kalman_IMU.h"
#include "IMU.h"
#include "PID.h"
#include "RX.h"
#include "ESC.h"
#include "MPU6050.h"
#include "HMC5883L.h"
#include "MS5611.h"
#include "FUNCTIONS.h"

Kalman_IMU roll, pitch, yaw;

int main1(void){
	SystemInit();
	TIMERbegin();
	UARTbegin(57600);
	PWMbegin();
	TWIbegin();
	//Set the I2C clock speed to 400kHz.
	TWI_ConfigureMaster(TWI1, 400000UL, 84000000UL);
	ADCbegin();
	init_all_pins();
	
	init_ms5611(MS5611_OSR_1024);
	_delay_ms(100);
	init_altitude();
	_delay_ms(100);
	while(1){
		calculate_compensated_pressure();
		calculate_altitude();
		print(P);
		tabspace();
		print(TEMP);
		tabspace();
		println(alt*100);
		_delay_ms(50);
	}
}

int main(void)
{
    SystemInit();
	TIMERbegin();
	//UARTbegin(57600);
	PWMbegin();
	TWIbegin();
	//Set the I2C clock speed to 400kHz.
	TWI_ConfigureMaster(TWI1, 400000UL, 84000000UL);
	ADCbegin();
	init_all_pins();
	
	//The flight controller needs the MPU-6050 with gyro and accelerometer and HMC5883L with magnetometer
	//writeln("setting up");
	set_mpu_registers();                                                     //Set the specific gyro registers.
	//set_hmc_registers();                                                     //Set the specific compass registers.
    //writeln("setup done");
	
	for (cal_int = 0; cal_int < 1250 ; cal_int ++){                           //Wait 5 seconds before continuing.
		_delay_us(4000);                                                      //Wait 4000us.
	}
	
	set_pulse(1000, 1000, 1000, 1000);
    //print("calibrating");
	calibrate_mpu();
	//println("calibrated...");
		
	//initialize everything
	roll.init_main(dt, 0);
	pitch.init_main(dt, 0);
	yaw.init_main(dt, 0);
		
	roll.init_processCov(processCov_roll);
	pitch.init_processCov(processCov_pitch);
	yaw.init_processCov(processCov_yaw);
	
	roll.init_measureCov(0);
	pitch.init_measureCov(0);
	yaw.init_measureCov(0);
	
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//setup NVIC
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	setup_interrupts();
	
	wait_until_rx_is_active();	
	
	load_lipo_voltage();
	
	//When everything is done, turn off the led.
	PIOB->PIO_CODR |= r_led;                                                      //Turn off the warning led.
	PIOD->PIO_SODR |= g_led;                                                      //Turn on green led.
	
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//Main flight controller program loop begins here.
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	loop_timer = micros();                                                        //Set the timer for the next loop.
    while (1){
		//Complimentary Filter to reduce Noise
		//Calculating input for PID Controller
		gyro_roll_input  = (gyro_roll_input * 0.7) + (gyro_x * 0.3);              //Gyro pid input is deg/sec.
		gyro_pitch_input = (gyro_pitch_input * 0.7) + (gyro_y * 0.3);             //Gyro pid input is deg/sec.
		gyro_yaw_input   = (gyro_yaw_input * 0.7) + (gyro_z * 0.3);               //Gyro pid input is deg/sec.

		//Calculating input for PID Controller
		if(altitude_hold) baro_alt_input = pid_alt_factor*alt;
		
		////////////////////////////////////////////////////////////////////////////////////////////////////
		//The IMU code begins here.
		////////////////////////////////////////////////////////////////////////////////////////////////////
		
		//Accelerometer Euler angle calculations
		cal_acc_euler(acc_x, acc_y, acc_z);                                       //updates angle_roll_acc & angle_pitch_acc variables
		
		//Gyroscope Euler rate Calculations
		relative_to_euler(gyro_x, gyro_y, gyro_z);                                //updates rate_roll, rate_pitch & rate_yaw variables
		
		//Magnetometer Euler angle calculations
		//cal_mag_yaw(comp_x, comp_y, comp_z);                                      //updates angle_yaw_com variable
		
		//Accelerometer offset calibration values.
		angle_pitch_acc -= 0.0;                                                   //Accelerometer calibration value for pitch.
		angle_roll_acc -= 0.0;                                                    //Accelerometer calibration value for roll.
		
		//Calculate variance
		roll_buffer_calculations();
		pitch_buffer_calculations();
		//yaw_buffer_calculations();
		
		roll.kalmanOutput(angle_roll_acc, rate_roll, angle_var_roll, rate_var_roll);
		pitch.kalmanOutput(angle_pitch_acc, rate_pitch, angle_var_roll, rate_var_pitch);
		//yaw.kalmanOutput(angle_yaw_com, rate_yaw, angle_var_yaw, rate_var_yaw);
		
		////////////////////////////////////////////////////////////////////////////////////////////////////
		//This is the end of IMU code.
		////////////////////////////////////////////////////////////////////////////////////////////////////
				
		ARM_or_DISARM_motors();
				
		////////////////////////////////////////////////////////////////////////////////////////////////////
		//The PID code begins here.
		////////////////////////////////////////////////////////////////////////////////////////////////////
		
		calculate_pid_level_adjust();                                               
		calculate_pid_setpoints();                                                  	
		calculate_pid();                                                            //PID inputs are known. So we can calculate the pid output.
		if(altitude_hold) calculate_pid_alt();
		else pid_output_alt = 0;
		
		////////////////////////////////////////////////////////////////////////////////////////////////////
		//This is the end of PID code.
		////////////////////////////////////////////////////////////////////////////////////////////////////

		//The battery voltage is needed for compensation.
		//A complementary filter is used to reduce noise.
		//0.0012308 = 0.004 * 0.3077.
		battery_voltage = battery_voltage * 0.996 + analogReadA0() * 0.0012308;

		//Turn on the led if battery voltage is too low.
		if(battery_voltage < 1000 && battery_voltage > 600){
			battery_low = true;
			PIOD->PIO_CODR |= g_led;
			PIOD->PIO_CODR |= b_led;
			PIOB->PIO_SODR |= r_led;
		}


		calculate_ESC_pulses();
		
		//! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !//
		//Because of the angle calculation the loop time is getting very important. If the loop time is  //
		//longer or shorter than 4000us the angle calculation is off. If you modify the code make sure   //
		//that the loop time is still 4000us and no longer!                                              //
		//! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !//
		
		if(micros() - loop_timer > 4050){
			PIOD->PIO_CODR |= g_led;
			PIOD->PIO_CODR |= b_led;
			PIOB->PIO_SODR |= r_led;                   //Turn on the LED if the loop time exceeds 4050us.
		}
		
		//All the information for controlling the motor's is available.
		//The refresh rate is 250Hz. That means the ESC's need there pulse every 4ms.
		while(micros() - loop_timer < 4000);                                      //We wait until 4000us are passed.
		loop_timer = micros();                                                    //Set the timer for the next loop.

		//Set digital ESC outputs high.
		/*
		 * ESC 1 - CCW - - Right Front
		 * ESC 2 - CW  - - Right Rear
		 * ESC 3 - CCW - - Left Rear
		 * ESC 4 - CW  - - Left Front
		 */
		
		//Send calculated pulse to the ESCs
		set_pulse(esc_1, esc_2, esc_3, esc_4);
		
		//Get the current gyro, accelerometer, compass and receiver data and scale it to degrees per second for the PID calculations.
		read_mpu_data();
		//read_hmc_data();
		
		read_rx_input();
		
	}// end while()
}//end main() 


