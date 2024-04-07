#ifndef IMU
#define IMU

#include "Kalman_IMU.h"

extern Kalman_IMU roll, pitch, yaw;

//IMU variables
double head_direction = 0;
short temperature;
short acc_axis[4], gyro_axis[4], comp_axis[4];
double gyro_axis_cal[4];
int cal_int;
double gyro_y, gyro_x, gyro_z;
double comp_x, comp_y, comp_z;
long acc_x, acc_y, acc_z, acc_total_vector;
double angle_roll_acc, angle_pitch_acc, angle_yaw_com;
double rate_roll, rate_pitch, rate_yaw;

/////////////////////////////////variables for variance calculation//////////////////////////////////////////////////
uint8_t index = 0;
const uint8_t num = 5;
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

void cal_mag_yaw(double comp_x, double comp_y, double comp_z){
	double numerator = comp_z*sin(roll.angle) - comp_y*cos(roll.angle);
	double denominator = comp_x*cos(pitch.angle) + (comp_y*sin(roll.angle) + comp_z*cos(roll.angle))*sin(pitch.angle);
	
	//https://en.wikipedia.org/wiki/Atan2
	angle_yaw_com = atan2(numerator,denominator) * 57.29578;
	
}

void cal_acc_euler(double acc_x, double acc_y, double acc_z){
	acc_x -= 0;
	acc_y -= 0;
	acc_z -= 0;
	
	acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));         //Calculate the total accelerometer vector.
	
	if(abs(acc_y) < acc_total_vector){                                          //Prevent the asin function to produce a NaN
		angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.29578;          //Calculate the pitch angle.
	}
	if(abs(acc_x) < acc_total_vector){                                          //Prevent the asin function to produce a NaN
		angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.29578;          //Calculate the roll angle.
	}
}

#endif