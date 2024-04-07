#ifndef PID
#define PID

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

//altitude hold
float pid_p_gain_alt = -10;               //Gain setting for the alt P-controller
float pid_i_gain_alt = 0.0;              //Gain setting for the alt I-controller
float pid_d_gain_alt = 0.0;              //Gain setting for the alt D-controller
int pid_max_alt = 100;                    //Maximum output of the PID-controller (+/-)
float pid_alt_factor = 100;


//PID variables
float roll_level_adjust, pitch_level_adjust;
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
//altitude hold
float pid_i_mem_alt, pid_alt_setpoint, baro_alt_input, pid_output_alt, pid_last_alt_d_error;

void calculate_pid(void){
	
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

void calculate_pid_alt(void){
	
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//Subroutine for calculating PID outputs
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	//Altitude Hold calculations
	pid_error_temp = pid_alt_setpoint - baro_alt_input;
	pid_i_mem_alt += pid_i_gain_alt * pid_error_temp;
	if(pid_i_mem_alt > pid_max_alt)pid_i_mem_alt = pid_max_alt;
	else if(pid_i_mem_alt < pid_max_alt * -1)pid_i_mem_alt = pid_max_alt * -1;

	pid_output_alt = pid_p_gain_alt * pid_error_temp + pid_i_mem_alt + pid_d_gain_alt * (pid_error_temp - pid_last_alt_d_error);
	if(pid_output_alt > pid_max_alt)pid_output_alt = pid_max_alt;
	else if(pid_output_alt < pid_max_alt * -1)pid_output_alt = pid_max_alt * -1;

	pid_last_alt_d_error = pid_error_temp;

}


#endif  