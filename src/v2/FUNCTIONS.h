#ifndef FUNCIONS
#define FUNCTIONS

void startup_routine(void){
	
	head_direction = yaw.angle;                                     //Store the head direction.
	yaw.init_main(dt, 0);                                           //Set the gyro yaw angle equal to the accelerometer roll angle when the quadcopter is started.
	
	//Reset the PID controllers for a bump less start.
	pid_i_mem_roll = 0;
	pid_last_roll_d_error = 0;
	pid_i_mem_pitch = 0;
	pid_last_pitch_d_error = 0;
	pid_i_mem_yaw = 0;
	pid_last_yaw_d_error = 0;
}

void init_all_pins(void){
	//Initialize LED pins as Output
	//13  12  11
	//B27 D8  D7
	PIOB->PIO_OER |= r_led;
	PIOD->PIO_OER |= g_led|b_led;
	
	PIOB->PIO_CODR |= r_led;
	PIOD->PIO_CODR |= g_led|b_led;

	//Use the led on the Arduino for startup indication.
	PIOB->PIO_SODR |= r_led;                                           //Turn on the warning led.

}

void ARM_or_DISARM_motors(void){
	
	//For starting the motors: throttle low and yaw left (step 1).
	if(receiver_input_channel_3 < 1050 && receiver_input_channel_4 < 1050)start = 1;
	//When yaw stick is back in the center position start the motors (step 2).
	if(start == 1 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1450){
		start = 2;
		startup_routine();
	}
	//Stopping the motors: throttle low and yaw right.
	if(start == 2 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1950)start = 0;

}

void calculate_pid_level_adjust(void){
	
	pitch_level_adjust = pitch.angle * 15;                                    //Calculate the pitch angle correction
	roll_level_adjust = roll.angle * 15;                                      //Calculate the roll angle correction

	if(!auto_level){                                                          //If the quadcopter is not in auto-level mode
		pitch_level_adjust = 0;                                               //Set the pitch angle correction to zero.
		roll_level_adjust = 0;                                                //Set the roll angle correction to zero.
	}
}

void calculate_pid_setpoints(void){
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
}

void read_rx_input(void){
	rec_1 = convert_receiver_channel(1);                 //Convert the actual receiver signals for pitch to the standard 1000 - 2000us.
	rec_2 = convert_receiver_channel(2);                 //Convert the actual receiver signals for roll to the standard 1000 - 2000us.
	rec_3 = convert_receiver_channel(3);                 //Convert the actual receiver signals for throttle to the standard 1000 - 2000us.
	rec_4 = convert_receiver_channel(4);                 //Convert the actual receiver signals for yaw to the standard 1000 - 2000us.
	rec_5 = convert_receiver_channel(5);
	rec_6 = convert_receiver_channel(6);
	
	//Activate headless mode if channel 5 gives 2000us pulse
	if(rec_5 > 1900) headless_mode = true;
	else if(rec_5 > 900 && rec_5 < 1100) headless_mode = false;
	
	//Activate altitude hold mode if channel 6 gives 2000us pulse
	if((rec_6 > 1900) && !battery_low){
		altitude_hold = true;
		PIOB->PIO_CODR |= r_led;
		PIOD->PIO_CODR |= g_led;
		PIOD->PIO_SODR |= b_led;
	}
	else if(rec_6 > 900 && rec_6 < 1100){
		altitude_hold = false;
		alt_hold_setpoint_set = false;
		if(!battery_low){
			PIOB->PIO_CODR |= r_led;
			PIOD->PIO_CODR |= b_led;
			PIOD->PIO_SODR |= g_led;
		}
	}
	
	if(headless_mode){
		rec_1 -= 1500;
		rec_2 -= 1500;
		
		double norm = 0.7071067812;
		//0.01745329251 = (3.142(PI) / 180degr) The sin function takes in radians
		double psi = (yaw.angle - head_direction) * 0.01745329251;
		
		receiver_input_channel_1 = (cos(psi)*rec_1 - sin(psi)*rec_2)*norm + 1500;
		receiver_input_channel_2 = (cos(psi)*rec_2 + sin(psi)*rec_1)*norm + 1500;
		receiver_input_channel_3 = rec_3;
		receiver_input_channel_4 = rec_4;
	}
	
	
	if(altitude_hold && !alt_hold_setpoint_set){
		pid_alt_setpoint = pid_alt_factor*alt;
		alt_hold_setpoint_set = true;
	}
	
	else{
		receiver_input_channel_1 = rec_1;
		receiver_input_channel_2 = rec_2;
		receiver_input_channel_3 = rec_3;
		receiver_input_channel_4 = rec_4;
	}
	
}

void wait_until_rx_is_active(void){
	//Wait until the receiver is active and the throttle is set to the lower position.
	while(receiver_input_channel_3 < 990 || receiver_input_channel_3 > 1020 || receiver_input_channel_4 < 1400){
		receiver_input_channel_3 = convert_receiver_channel(3);                 //Convert the actual receiver signals for throttle to the standard 1000 - 2000us
		receiver_input_channel_4 = convert_receiver_channel(4);                 //Convert the actual receiver signals for yaw to the standard 1000 - 2000us
		start ++;                                                               //While waiting increment start with every loop.
		//We don't want the ESC's to be beeping annoyingly. So let's give them a 1000us pulse while waiting for the receiver inputs.
		_delay_us(4000);                                                        //Wait 4000us.
		
		if(start == 125){                                                       //Every 125 loops (500ms).
			//Change the led status.
			if((PIOB->PIO_ODSR & r_led) == r_led) PIOB->PIO_CODR |= r_led;
			else PIOB->PIO_SODR |= r_led;
			start = 0;                                                            //Start again at 0.
		}
	}
	start = 0;                                                                    //Set start back to 0.
}

void load_lipo_voltage(void){
	//Load the battery voltage to the battery_voltage variable.
	//12.6V equals ~3.3V @ Analog 0.
	//12.6V equals 4095 analogRead(0).
	//1260 / 4095 = 0.3077.
	//The variable battery_voltage holds 1050 if the battery voltage is 10.5V.
	//battery_voltage = analogReadA0() * 0.3077;
	for (int i=0; i<10;  i++){
		battery_voltage = analogReadA0();
		_delay_us(500);
	}
}

#endif