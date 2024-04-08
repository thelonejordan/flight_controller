#ifndef HMC
#define HMC

#define hmc_address 0x1E

void set_hmc_registers(void){
	//Setup the HMC5883L
	
	//We want to write to the CRA register (00 hex)
	//No. of samples averaged per measurement: 4
	//Typical Data Output Rate (Hz): 75
	//Incorporated normal measurement mode
	TWI_StartWrite(TWI1, hmc_address, 0x00, 1, 0x58 );
	TWI_Stop(TWI1);
	TWI_WaitByteSent(TWI1, XMIT_TIMEOUT);
	TWI_WaitTransferComplete(TWI1, XMIT_TIMEOUT);

	//We want to write to the CRB register (01 hex)
	//Sensor Field Range: +-1.3 Ga
	//Gain: 1090 LSB/Ga
	TWI_StartWrite(TWI1, hmc_address, 0x01, 1, 0x20 );
	TWI_Stop(TWI1);
	TWI_WaitByteSent(TWI1, XMIT_TIMEOUT);
	TWI_WaitTransferComplete(TWI1, XMIT_TIMEOUT);

	//We want to write to the MODE register (02 hex)
	//Continuous-Measurement Mode enabled
	//High Speed i2c enabled
	TWI_StartWrite(TWI1, hmc_address, 0x02, 1, 0x80 );
	TWI_Stop(TWI1);
	TWI_WaitByteSent(TWI1, XMIT_TIMEOUT);
	TWI_WaitTransferComplete(TWI1, XMIT_TIMEOUT);

}

void read_hmc_data(void){
	//Read the HMC5883L

	//Start reading @ register 03h and auto increment with every read.
	TWI_StartRead(TWI1, hmc_address, 0x03, 1);
	
	TWI_WaitByteReceived(TWI1, RECV_TIMEOUT);
	comp_axis[1] = TWI_ReadByte(TWI1)<<8;
	TWI_WaitByteReceived(TWI1, RECV_TIMEOUT);
	comp_axis[1] |= TWI_ReadByte(TWI1);
	
	TWI_WaitByteReceived(TWI1, RECV_TIMEOUT);
	comp_axis[2] = TWI_ReadByte(TWI1)<<8;
	TWI_WaitByteReceived(TWI1, RECV_TIMEOUT);
	comp_axis[2] |= TWI_ReadByte(TWI1);
	
	TWI_WaitByteReceived(TWI1, RECV_TIMEOUT);
	comp_axis[3] = TWI_ReadByte(TWI1)<<8;
	TWI_Stop(TWI1);
	TWI_WaitByteReceived(TWI1, RECV_TIMEOUT);
	comp_axis[3] |= TWI_ReadByte(TWI1);
	TWI_WaitTransferComplete(TWI1, RECV_TIMEOUT);
	
	comp_x = comp_axis[2];
	comp_y = comp_axis[1];
	comp_z = comp_axis[3];
}




#endif