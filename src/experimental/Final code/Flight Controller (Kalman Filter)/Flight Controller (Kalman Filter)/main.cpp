/*
 * Flight Controller (Kalman Filter).cpp
 *
 * Created: 08-07-2017 11:32:04
 * Author : Jyotirmaya Mahanta
 */ 


#include <math.h>

#include "sam3x8e.h"
#include "InitTimer.h"
#include "InitTWI.h"
#include "InitUART.h"
#include "InitADC.h"
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
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 1.3;               //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.04;              //Gain setting for the roll I-controller
float pid_d_gain_roll = 18.0;              //Gain setting for the roll D-controller
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 4.0;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)

bool auto_level = true;                 //Auto level on (true) or off (false)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void yaw_buffer_calculations(void);
void pitch_buffer_calculations(void);
void roll_buffer_calculations(void);
void read_com_data(void);
//void set_com_registers(void);
void relative_to_euler(double w_x, double w_y, double w_z);
void calibrate_mpu(void);
void init_all_pins(void);
void startup_routine(void);
void setup_interrupts(void);
void read_mpu_data(void);
void calculate_pid(void);
int  convert_receiver_channel(uint8_t function);
void set_mpu_registers(void);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Output for ESCs
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, loop_counter;
int esc_1, esc_2, esc_3, esc_4;
int throttle, battery_voltage;
int start;
int receiver_input[7];

//Receiver input variables
uint8_t last_channel[7];
volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
uint32_t timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
uint32_t timer[7], current_time;

uint32_t loop_timer;

//PID variables
float roll_level_adjust, pitch_level_adjust;
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;

//IMU variables
short temperature;
short acc_axis[4], gyro_axis[4];
double gyro_axis_cal[4];
int cal_int;
double gyro_y, gyro_x, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector;
double angle_roll_acc, angle_pitch_acc, angle_yaw_com;
double rate_roll, rate_pitch, rate_yaw;
bool gyro_angles_set;

//  01 02 03 04 05 06
//  33 35 37 39 41 40
//  C1 C3 C5 C7 C9 C8
uint32_t rx_pins = (1u<<1)|(1u<<3)|(1u<<5)|(1u<<7)|(1u<<8)|(1u<<9);
//  01  02  03  04
//  50  48  46  44
//  C13 C15 C17 C19
uint32_t esc_pins = (1u<<13)|(1u<<15)|(1u<<17)|(1u<<19);
uint32_t status;
uint32_t esc_status = 0;

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

int main(void)
{
    SystemInit();
	TIMERbegin();
	//UARTbegin(57600);
	TWIbegin();
	//Set the I2C clock speed to 400kHz.
	TWI_ConfigureMaster(TWI1, 400000UL, 84000000UL);
	ADCbegin();
	init_all_pins();
	//The flight controller needs the MPU-6050 with gyro and accelerometer
	//writeln("setting up");
	set_mpu_registers();                                                     //Set the specific gyro registers.
    //writeln("setup gyro done");
	for (cal_int = 0; cal_int < 1250 ; cal_int ++){                           //Wait 5 seconds before continuing.
		PIOC->PIO_SODR |= esc_pins;
		_delay_us(1000);                                                      //Wait 1000us.
		PIOC->PIO_CODR |= esc_pins;
		_delay_us(3000);                                                      //Wait 3000us.
	}
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
	
	//Wait until the receiver is active and the throttle is set to the lower position.
	while(receiver_input_channel_3 < 990 || receiver_input_channel_3 > 1020 || receiver_input_channel_4 < 1400){
		receiver_input_channel_3 = convert_receiver_channel(3);                 //Convert the actual receiver signals for throttle to the standard 1000 - 2000us
		receiver_input_channel_4 = convert_receiver_channel(4);                 //Convert the actual receiver signals for yaw to the standard 1000 - 2000us
		start ++;                                                               //While waiting increment start with every loop.
		//We don't want the ESC's to be beeping annoyingly. So let's give them a 1000us pulse while waiting for the receiver inputs.
		PIOC->PIO_SODR |= esc_pins;
		_delay_us(1000);                                                //Wait 1000us.
		PIOC->PIO_CODR |= esc_pins;
		_delay_us(3000);                                                //Wait 3000us.
		
		if(start == 125){                                                       //Every 125 loops (500ms).
			//Change the led status.
			if((PIOB->PIO_ODSR & PIO_ODSR_P27) == PIO_ODSR_P27) PIOB->PIO_CODR |= PIO_CODR_P27;
			else PIOB->PIO_SODR |= PIO_SODR_P27;
			start = 0;                                                            //Start again at 0.
		}
	}
	start = 0;                                                                //Set start back to 0.

	//Load the battery voltage to the battery_voltage variable.
	//12.6V equals ~3.3V @ Analog 0.
	//12.6V equals 4095 analogRead(0).
	//1260 / 4095 = 0.3077.
	//The variable battery_voltage holds 1050 if the battery voltage is 10.5V.
	//battery_voltage = analogReadA0() * 0.3077;
	battery_voltage = 1111;
	
	//When everything is done, turn off the led.
	PIOB->PIO_CODR |= PIO_CODR_P27;                                                     //Turn off the warning led.
	
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//Main flight controller program loop
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	loop_timer = micros();                                                    //Set the timer for the next loop.
    while (1){
		//Complimentary Filter to reduce Noise
		gyro_roll_input = (gyro_roll_input * 0.7) + (gyro_x * 0.3);   //Gyro pid input is deg/sec.
		gyro_pitch_input = (gyro_pitch_input * 0.7) + (gyro_y * 0.3);//Gyro pid input is deg/sec.
		gyro_yaw_input = (gyro_yaw_input * 0.7) + (gyro_z * 0.3);      //Gyro pid input is deg/sec.


		////////////////////////////////////////////////////////////////////////////////////////////////////
		//This is the IMU code 
		////////////////////////////////////////////////////////////////////////////////////////////////////
		
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
		
		//roll.kalmanOutput(angle_avg_roll, rate_roll, angle_var_roll, rate_var_roll);
		//pitch.kalmanOutput(angle_avg_pitch, rate_pitch, angle_var_roll, rate_var_pitch);
		
		roll.kalmanOutput(angle_roll_acc, rate_roll, angle_var_roll, rate_var_roll);
		pitch.kalmanOutput(angle_pitch_acc, rate_pitch, angle_var_roll, rate_var_pitch);
		//yaw.kalmanOutput(angle_yaw_com, rate_yaw, angle_var_yaw, rate_var_yaw);
		
		////////////////////////////////////////////////////////////////////////////////////////////////////
		//This is the end of IMU code
		////////////////////////////////////////////////////////////////////////////////////////////////////

		pitch_level_adjust = pitch.angle * 15;                                    //Calculate the pitch angle correction
		roll_level_adjust = roll.angle * 15;                                      //Calculate the roll angle correction

		if(!auto_level){                                                          //If the quadcopter is not in auto-level mode
			pitch_level_adjust = 0;                                                 //Set the pitch angle correction to zero.
			roll_level_adjust = 0;                                                  //Set the roll angle correction to zero.
		}
		
		//gyro_roll_input = (gyro_roll_input * 0.7) + (angle_roll * 1.5);
		//gyro_pitch_input = (gyro_pitch_input * 0.7) + (angle_pitch * 1.5);

		//For starting the motors: throttle low and yaw left (step 1).
		if(receiver_input_channel_3 < 1050 && receiver_input_channel_4 < 1050)start = 1;
		//When yaw stick is back in the center position start the motors (step 2).
		if(start == 1 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1450){
			start = 2;
			startup_routine();
		}
		//Stopping the motors: throttle low and yaw right.
		if(start == 2 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1950)start = 0;

		//The PID set point in degrees per second is determined by the roll receiver input.
		//In the case of dividing by 3 the max roll rate is approx 164 degrees per second ( (500-8)/3 = 164d/s ).
		pid_roll_setpoint = 0;
		//We need a little dead band of 16us for better results.
		if(receiver_input_channel_1 > 1508)pid_roll_setpoint = receiver_input_channel_1 - 1508;
		else if(receiver_input_channel_1 < 1492)pid_roll_setpoint = receiver_input_channel_1 - 1492;

		pid_roll_setpoint -= roll_level_adjust;                                   //Subtract the angle correction from the standardized receiver roll input value.
		pid_roll_setpoint /= 3.0;                                                 //Divide the set point for the PID roll controller by 3 to get angles in degrees.


		//The PID set point in degrees per second is determined by the pitch receiver input.
		//In the case of dividing by 3 the max pitch rate is approx 164 degrees per second ( (500-8)/3 = 164d/s ).
		pid_pitch_setpoint = 0;
		//We need a little dead band of 16us for better results.
		if(receiver_input_channel_2 > 1508)pid_pitch_setpoint = receiver_input_channel_2 - 1508;
		else if(receiver_input_channel_2 < 1492)pid_pitch_setpoint = receiver_input_channel_2 - 1492;

		pid_pitch_setpoint -= pitch_level_adjust;                                  //Subtract the angle correction from the standardized receiver pitch input value.
		pid_pitch_setpoint /= 3.0;                                                 //Divide the set point for the PID pitch controller by 3 to get angles in degrees.

		//The PID set point in degrees per second is determined by the yaw receiver input.
		//In the case of dividing by 3 the max yaw rate is approx 164 degrees per second ( (500-8)/3 = 164d/s ).
		pid_yaw_setpoint = 0;
		//We need a little dead band of 16us for better results.
		if(receiver_input_channel_3 > 1050){ //Do not yaw when turning off the motors.
			if(receiver_input_channel_4 > 1508)pid_yaw_setpoint = (receiver_input_channel_4 - 1508)/3.0;
			else if(receiver_input_channel_4 < 1492)pid_yaw_setpoint = (receiver_input_channel_4 - 1492)/3.0;
		}
		
		calculate_pid();                                                            //PID inputs are known. So we can calculate the pid output.

		//The battery voltage is needed for compensation.
		//A complementary filter is used to reduce noise.
		//0.0012308 = 0.004 * 0.3077.
		battery_voltage = battery_voltage * 0.996 + analogReadA0() * 0.0012308;

		//Turn on the led if battery voltage is too low.
		if(battery_voltage < 1000 && battery_voltage > 600)PIOB->PIO_SODR |= PIO_SODR_P27;


		throttle = receiver_input_channel_3;                                      //We need the throttle signal as a base signal.

		if (start == 2){                                                          //The motors are started.
			if (throttle > 1800) throttle = 1800;                                   //We need some room to keep full control at full throttle.
			esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
			esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
			esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
			esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)

			if (battery_voltage < 1240 && battery_voltage > 800){                   //Is the battery connected?
				esc_1 += esc_1 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-1 pulse for voltage drop.
				esc_2 += esc_2 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-2 pulse for voltage drop.
				esc_3 += esc_3 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-3 pulse for voltage drop.
				esc_4 += esc_4 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-4 pulse for voltage drop.
			}

			if (esc_1 < 1100) esc_1 = 1100;                                         //Keep the motors running.
			if (esc_2 < 1100) esc_2 = 1100;                                         //Keep the motors running.
			if (esc_3 < 1100) esc_3 = 1100;                                         //Keep the motors running.
			if (esc_4 < 1100) esc_4 = 1100;                                         //Keep the motors running.

			if(esc_1 > 2000)esc_1 = 2000;                                           //Limit the esc-1 pulse to 2000us.
			if(esc_2 > 2000)esc_2 = 2000;                                           //Limit the esc-2 pulse to 2000us.
			if(esc_3 > 2000)esc_3 = 2000;                                           //Limit the esc-3 pulse to 2000us.
			if(esc_4 > 2000)esc_4 = 2000;                                           //Limit the esc-4 pulse to 2000us.
		}

		else{
			esc_1 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ESC 1.
			esc_2 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ESC 2.
			esc_3 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ESC 3.
			esc_4 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ESC 4.
		}

		
		//! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
		//Because of the angle calculation the loop time is getting very important. If the loop time is
		//longer or shorter than 4000us the angle calculation is off. If you modify the code make sure
		//that the loop time is still 4000us and no longer!
		//! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
		
		if(micros() - loop_timer > 4050)PIOB->PIO_SODR |= PIO_SODR_P27;                   //Turn on the LED if the loop time exceeds 4050us.
		
		//All the information for controlling the motor's is available.
		//The refresh rate is 250Hz. That means the ESC's need there pulse every 4ms.
		while(micros() - loop_timer < 4000);                                      //We wait until 4000us are passed.
		loop_timer = micros();                                                    //Set the timer for the next loop.

		//Set digital ESC outputs high.
		/*
		 * ESC 1 - CCW - 50 - c13 - Right Front
		 * ESC 2 - CW  - 48 - C15 - Right Rear
		 * ESC 3 - CCW - 46 - C17 - Left Rear
		 * ESC 4 - CW  - 44 - C19 - Left Front
		 */
		
		PIOC->PIO_SODR |= esc_pins;
		esc_status = 1;
		timer_channel_1 = esc_1 + loop_timer;                                     //Calculate the time of the falling edge of the ESC 1 pulse.
		timer_channel_2 = esc_2 + loop_timer;                                     //Calculate the time of the falling edge of the ESC 2 pulse.
		timer_channel_3 = esc_3 + loop_timer;                                     //Calculate the time of the falling edge of the ESC 3 pulse.
		timer_channel_4 = esc_4 + loop_timer;                                     //Calculate the time of the falling edge of the ESC 4 pulse.
		
		//There is always 1000us of spare time. So let's do something useful that is very time consuming.
		//Get the current gyro and receiver data and scale it to degrees per second for the PID calculations.
		read_mpu_data();
		
		receiver_input_channel_1 = convert_receiver_channel(1);                 //Convert the actual receiver signals for pitch to the standard 1000 - 2000us.
		receiver_input_channel_2 = convert_receiver_channel(2);                 //Convert the actual receiver signals for roll to the standard 1000 - 2000us.
		receiver_input_channel_3 = convert_receiver_channel(3);                 //Convert the actual receiver signals for throttle to the standard 1000 - 2000us.
		receiver_input_channel_4 = convert_receiver_channel(4);                 //Convert the actual receiver signals for yaw to the standard 1000 - 2000us.

		while(esc_status){																		//Stay in this loop until output 4,5,6 and 7 are low.
			esc_loop_timer = micros();                                                          //Read the current time.
			if(timer_channel_1 <= esc_loop_timer)PIOC->PIO_CODR |= PIO_CODR_P13;                //Set digital ESC output 1 to low if the time is expired.
			if(timer_channel_2 <= esc_loop_timer)PIOC->PIO_CODR |= PIO_CODR_P15;                //Set digital ESC output 2 to low if the time is expired.
			if(timer_channel_3 <= esc_loop_timer)PIOC->PIO_CODR |= PIO_CODR_P17;                //Set digital ESC output 3 to low if the time is expired.
			if(timer_channel_4 <= esc_loop_timer)PIOC->PIO_CODR |= PIO_CODR_P19;                //Set digital ESC output 4 to low if the time is expired.
			esc_status = PIOC->PIO_ODSR & esc_pins;
		}
		
	}
}

void PIOC_Handler(){
	
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//This routine is called every time input 8, 9, 10 or 11 changed state. This is used to read the receiver signals.
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	current_time = micros();
	status = PIOC->PIO_PDSR;
	
	//channel 1 ====================================================================================
	if (status & (1u<<1)){
		if(last_channel[1] == 0){
			timer[1] = current_time;
			last_channel[1] = 1;
			PIOC->PIO_ISR;
		}
	}
	else if (last_channel[1] == 1){
		receiver_input[1] = current_time - timer[1];
		last_channel[1] = 0;
		PIOC->PIO_ISR;
	}
	
	//channel 2 =====================================================================================
	if (status & (1u<<3)){
		if(last_channel[2] == 0){
			timer[2] = current_time;
			last_channel[2] = 1;
			PIOC->PIO_ISR;
		}
	}
	else if (last_channel[2] == 1){
		receiver_input[2] = current_time - timer[2];
		last_channel[2] = 0;
		PIOC->PIO_ISR;
	}
	
	//channel 3 ======================================================================================
	if (status & (1u<<5)){
		if(last_channel[3] == 0){
			timer[3] = current_time;
			last_channel[3] = 1;
			PIOC->PIO_ISR;
		}
	}
	else if (last_channel[3] == 1){
		receiver_input[3] = current_time - timer[3];
		last_channel[3] = 0;
		PIOC->PIO_ISR;
	}
	
	//channel 4 ======================================================================================
	if (status & (1u<<7)){
		if(last_channel[4] == 0){
			timer[4] = current_time;
			last_channel[4] = 1;
			PIOC->PIO_ISR;
		}
	}
	else if (last_channel[4] == 1){
		receiver_input[4] = current_time - timer[4];
		last_channel[4] = 0;
		PIOC->PIO_ISR;
	}
	
	//channel 5 ======================================================================================
	if (status & (1u<<9)){
		if(last_channel[5] == 0){
			timer[5] = current_time;
			last_channel[5] = 1;
			PIOC->PIO_ISR;
		}
	}
	else if (last_channel[5] == 1){
		receiver_input[5] = current_time - timer[5];
		last_channel[5] = 0;
		PIOC->PIO_ISR;
	}
	
	//channel 6 ======================================================================================
	if (status & (1u<<8)){
		if(last_channel[6] == 0){
			timer[6] = current_time;
			last_channel[6] = 1;
			PIOC->PIO_ISR;
		}
	}
	
	else if (last_channel[6] == 1){
		receiver_input[6] = current_time - timer[6];
		last_channel[6] = 0;
		PIOC->PIO_ISR;
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
	
	gyro_x = gyro_axis[1]/65.5;                      //Set gyro_roll to the correct axis that was stored in the EEPROM.
	//gyro_x *= -1;                             //Invert gyro_roll if the MSB of EEPROM bit 28 is set.
	gyro_y = gyro_axis[2]/65.5;                     //Set gyro_pitch to the correct axis that was stored in the EEPROM.
	gyro_y *= -1;                              //Invert gyro_pitch if the MSB of EEPROM bit 29 is set.
	gyro_z = gyro_axis[3]/65.5;                       //Set gyro_yaw to the correct axis that was stored in the EEPROM.
	gyro_z *= -1;                                //Invert gyro_yaw if the MSB of EEPROM bit 30 is set.

	acc_x = acc_axis[2];                           //Set acc_x to the correct axis that was stored in the EEPROM.
	acc_x *= -1;                                   //Invert acc_x if the MSB of EEPROM bit 29 is set.
	acc_y = acc_axis[1];                           //Set acc_y to the correct axis that was stored in the EEPROM.
	//acc_y *= -1;                                 //Invert acc_y if the MSB of EEPROM bit 28 is set.
	acc_z = acc_axis[3];                           //Set acc_z to the correct axis that was stored in the EEPROM.
	acc_z *= -1;                                   //Invert acc_z if the MSB of EEPROM bit 30 is set.
	
}

void calculate_pid(){
	
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//Subroutine for calculating PID outputs
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	//Roll calculations
	pid_error_temp = gyro_roll_input - pid_roll_setpoint;
	pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
	if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
	else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

	pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
	if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
	else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

	pid_last_roll_d_error = pid_error_temp;

	//Pitch calculations
	pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
	pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
	if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
	else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

	pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
	if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
	else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

	pid_last_pitch_d_error = pid_error_temp;

	//Yaw calculations
	pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
	pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
	if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
	else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

	pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
	if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
	else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

	pid_last_yaw_d_error = pid_error_temp;
}

int convert_receiver_channel(uint8_t function){
	//This part converts the actual receiver signals to a standardized 1000 – 1500 – 2000 microsecond value.
	
	int low=0, center=0, high=0, actual=0;
	int difference=0;

	//roll
	if(function==1){
		low = 1000;
		center = 1500;
		high = 2010;
	}
	//pitch
	else if(function==2){
		low = 1000;
		center = 1500;
		high = 2006;
	}
	//throttle
	else if(function==3){
		low = 1000;
		center = 1500;
		high = 2010;
	}
	//yaw
	else if(function==4){
		low = 995;
		center = 1500;
		high = 2000;
	}
	//SWC
	else if(function==5){
		low = 1039;
		center = 1553;
		high = 2000;
	}
	//SWD
	else if(function==6){
		low = 1028;
		center = 1522;
		high = 2016;
	}
	
	actual = receiver_input[function];                                             //Read the actual receiver value for the corresponding function
	if(actual > 3000) actual -= 20000;
	
	if(actual < center){                                                           //The actual receiver value is lower than the center value
		if(actual < low)actual = low;                                              //Limit the lowest value to the value that was detected during setup
		difference = ((long)(center - actual) * (long)500) / (center - low);       //Calculate and scale the actual value to a 1000 - 2000us value
		if(function == 2)return 1500 + difference;                                 //If the channel is reversed
		else return 1500 - difference;                                             //If the channel is not reversed
	}
	else if(actual > center){                                                      //The actual receiver value is higher than the center value
		if(actual > high)actual = high;                                            //Limit the lowest value to the value that was detected during setup
		difference = ((long)(actual - center) * (long)500) / (high - center);      //Calculate and scale the actual value to a 1000 - 2000us value
		if(function == 2)return 1500 - difference;                                 //If the channel is reversed
		else return 1500 + difference;                                             //If the channel is not reversed
	}
	else return 1500;
}

void set_mpu_registers(){
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

void setup_interrupts(void){
	PMC->PMC_PCER0 |= (1u<<ID_PIOC);        //enable peripheral clock
	PIOC->PIO_IER |= rx_pins;               //enable pin change interrupts
	PIOC->PIO_PER |= rx_pins;               //enable PIO controller
	PIOC->PIO_ODR |= rx_pins;               //set as input
	NVIC_SetPriority(PIOC_IRQn, 0);			//set priority
	NVIC_EnableIRQ(PIOC_IRQn);              //configure NVIC
}

void startup_routine(void){
	roll.init_main(dt, angle_roll_acc);								//Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.
	pitch.init_main(dt, angle_pitch_acc);							//Set the gyro pitch angle equal to the accelerometer roll angle when the quadcopter is started.
	yaw.init_main(dt, 0);                                           //Set the gyro yaw angle equal to the accelerometer roll angle when the quadcopter is started.
	gyro_angles_set = true;                                         //Set the IMU started flag.

	//Reset the PID controllers for a bump less start.
	pid_i_mem_roll = 0;
	pid_last_roll_d_error = 0;
	pid_i_mem_pitch = 0;
	pid_last_pitch_d_error = 0;
	pid_i_mem_yaw = 0;
	pid_last_yaw_d_error = 0;
}

void init_all_pins(void){
	//46 48 50 52 - ESC channels - c17 c15 c13 b21
	//Initialize ESC pins as output
	PIOC->PIO_PER |= esc_pins;
	PIOC->PIO_OER |= esc_pins;
	
	//Initialize LED pins as Output
	//13  12  11
	//B27 D8  D7
	PIOB->PIO_OER |= PIO_OER_P27;
	PIOD->PIO_OER |= PIO_OER_P8|PIO_OER_P7;
	
	PIOB->PIO_CODR |= PIO_CODR_P27;
	PIOD->PIO_CODR |= PIO_CODR_P8|PIO_CODR_P7;

	//Use the led on the Arduino for startup indication.
	PIOB->PIO_SODR |= PIO_SODR_P27;                                           //Turn on the warning led.

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
		//We don't want the ESC's to be beeping annoyingly. So let's give them a 1000us pulse while calibrating the gyro.
		PIOC->PIO_SODR |= esc_pins;
		_delay_us(1000);                                                        //Wait 1000us.
		PIOC->PIO_CODR |= esc_pins;
		_delay_us(3000);                                                        //Wait 3000us.
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