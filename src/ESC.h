#ifndef ESC
#define ESC

//Output for ESCs
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, loop_counter;
int esc_1, esc_2, esc_3, esc_4;
int throttle, battery_voltage;
int start;


void compensate_4_lipo_voltage(void){
	if (battery_voltage < 1240 && battery_voltage > 800){                   //Is the battery connected?
		throttle += throttle * ((1240 - battery_voltage)/(float)3500);//Compensate the esc-1 pulse for voltage drop.
	}
}

void calculate_ESC_pulses(void){
	throttle = receiver_input_channel_3;                                        //We need the throttle signal as a base signal.

    if (start == 2){                                                            //The motors are started.

		compensate_4_lipo_voltage();
		pid_output_alt = 0;

		if (throttle > 1800) throttle = 1800;                                   //We need some room to keep full control at full throttle.
		esc_1 = throttle + pid_output_alt - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
		esc_2 = throttle + pid_output_alt + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
		esc_3 = throttle + pid_output_alt + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
		esc_4 = throttle + pid_output_alt - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)

		/*if (battery_voltage < 1240 && battery_voltage > 800){                   //Is the battery connected?
			esc_1 += esc_1 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-1 pulse for voltage drop.
			esc_2 += esc_2 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-2 pulse for voltage drop.
			esc_3 += esc_3 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-3 pulse for voltage drop.
	      }*/

		if (esc_1 < 1100) esc_1 = 1100;                                         //Keep the motors running.
		if (esc_2 < 1100) esc_2 = 1100;                                         //Keep the motors running.
		if (esc_3 < 1100) esc_3 = 1100;                                         //Keep the motors running.
		if (esc_4 < 1100) esc_4 = 1100;                                         //Keep the motors running.

		if(esc_1 > 2000)esc_1 = 2000;                                           //Limit the esc-1 pulse to 2000us.
		if(esc_2 > 2000)esc_2 = 2000;                                           //Limit the esc-2 pulse to 2000us.
		if(esc_3 > 2000)esc_3 = 2000;                                           //Limit the esc-3 pulse to 2000us.
		if(esc_4 > 2000)esc_4 = 2000;                                           //Limit the esc-4 pulse to 2000us.
	}

	else {
		esc_1 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ESC 1.
		esc_2 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ESC 2.
		esc_3 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ESC 3.
		esc_4 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ESC 4.
	}	
}

#endif