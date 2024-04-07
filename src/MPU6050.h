#ifndef MPU6050
#define MPU6050

#define mpu_address 0x68

void set_mpu_registers(void){
	//Setup the MPU-6050
	
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
		PIOB->PIO_SODR |= r_led;
		while(1) _delay_ms(10);
	}

	//We want to write to the CONFIG register (1A hex)
	//Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
	TWI_StartWrite(TWI1, mpu_address, 0x1A, 1, 0x03 );
	TWI_Stop(TWI1);
	TWI_WaitByteSent(TWI1, XMIT_TIMEOUT);
	TWI_WaitTransferComplete(TWI1, XMIT_TIMEOUT);
}

void read_mpu_data(void){
	//Read the MPU-6050
	
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

void calibrate_mpu(void){
	//Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
	for (cal_int = 0; cal_int < 2000 ; cal_int ++){                           //Take 2000 readings for calibration.
		if(cal_int % 15 == 0){
			//Change the led status to indicate calibration.
			if((PIOB->PIO_ODSR & r_led) == r_led) PIOB->PIO_CODR |= r_led;
			else PIOB->PIO_SODR |= r_led;
		}
		read_mpu_data();                                                        //Read the gyro output.
		//println(gyro_axis[1]);
		gyro_axis_cal[1] += gyro_axis[1];                                       //Add roll value to gyro_roll_cal.
		gyro_axis_cal[2] += gyro_axis[2];                                       //Add pitch value to gyro_pitch_cal.
		gyro_axis_cal[3] += gyro_axis[3];                                       //Add yaw value to gyro_yaw_cal.
		//We don't want the ESC's to be beeping annoyingly. So let's give them a 1000us pulse while calibrating the gyro.
		_delay_us(4000);                                                        //Wait 4000us.
	}
	//Now that we have 2000 measures, we need to divide by 2000 to get the average gyro offset.
	gyro_axis_cal[1] /= 2000;                                                 //Divide the roll total by 2000.
	gyro_axis_cal[2] /= 2000;                                                 //Divide the pitch total by 2000.
	gyro_axis_cal[3] /= 2000;                                                 //Divide the yaw total by 2000.
}


#endif