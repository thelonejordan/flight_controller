#ifndef MS5611
#define MS5611

//VCC---5V
//GND---GND
//SCL---SCL
//SDA---SDA
//PS ---5V          //activating I2C interface
//CSB---HIGH        //don't leave unconnected

#define MS5611_ADDR_CSB_HIGH  0x76   //CBR=1 0x76 I2C address when CSB is connected to HIGH (VCC)
#define MS5611_ADDR_CSB_LOW   0x77   //CBR=0 0x77 I2C address when CSB is connected to LOW (GND)

//barometer variables
double alt, alt_ground;
bool alt_ground_set = false;
int throttle_correction;

const uint8_t ms5611_address = MS5611_ADDR_CSB_HIGH;

// Commands
#define MS5611_CMD_ADC_READ           (0x00)
#define MS5611_CMD_RESET              (0x1E)
#define MS5611_CMD_CONV_D1            (0x40)
#define MS5611_CMD_CONV_D2            (0x50)
#define MS5611_CMD_READ_PROM          (0xA2)

// OSR (Over Sampling Ratio) constants
typedef enum
{
	MS5611_OSR_4096   = 0x08,
	MS5611_OSR_2048   = 0x06,
	MS5611_OSR_1024   = 0x04,
	MS5611_OSR_512    = 0x02,
	MS5611_OSR_256    = 0x00
} ms5611_osr_t;

// C1 will be at 0xA2 and all the subsequent are multiples of 2
#define MS5611_PROM_REG_SIZE 2 // size in bytes of a prom registry.

//Maximum values for calculation results:
//const int PMIN = 10;			// mbar
//const int PMAX = 1200;	    // mbar
//const int TMIN = -40;			// °C
//const int TMAX = 85;			// °C
//const int TREF = 20;			// °C

//Read calibration data (factory calibrated) from PROMRead
uint16_t C1;                    //Pressure sensitivity | SENS_T1
uint16_t C2;					//Pressure offset | OFF_T1
uint16_t C3;					//Temperature coefficient of pressure sensitivity | TCS
uint16_t C4;					//Temperature coefficient of pressure offset | TCO
uint16_t C5;					//Reference temperature | T_REF
uint16_t C6;					//Temperature coefficient of the temperature | TEMPSENS

// const uint16_t C1 = 45742;                    //Pressure sensitivity | SENS_T1
// const uint16_t C2 = 46480;					//Pressure offset | OFF_T1
// const uint16_t C3 = 28201;					//Temperature coefficient of pressure sensitivity | TCS
// const uint16_t C4 = 25410;					//Temperature coefficient of pressure offset | TCO
// const uint16_t C5 = 32849;					//Reference temperature | T_REF
// const uint16_t C6 = 27603;					//Temperature coefficient of the temperature | TEMPSENS

//Read digital pressure and temperature data
uint32_t D1;					//Digital pressure value
uint32_t D2;					//Digital temperature value

//Calculate temperature
int32_t dT;						//Difference between actual and reference temperature    | (reading in °C)*100
int32_t TEMP;					//Actual temperature(-40...85°C with 0.01°C resolution)  | reading in Pascals

//Calculate temperature compensated pressure
int64_t OFF;					//Offset at actual temperature
int64_t SENS;					//Sensitivity at actual temperature
int32_t P;						//Temperature compensated pressure (10...1200mbar with 0.01mbar resolution)

//Second order temperature compensation
bool do_second_order_temp_compensation = false;

unsigned long lastPresConv;
unsigned long lastTempConv;
unsigned long conversion_time;
ms5611_osr_t OSR;

int16_t read_register_16(uint8_t _i_address){
	int16_t c;
	uint8_t v1, v2;
	
	TWI_StartRead(TWI1, ms5611_address, _i_address, 1);
	TWI_WaitByteReceived(TWI1, RECV_TIMEOUT);
	v2 = TWI_ReadByte(TWI1);
	TWI_Stop(TWI1);
	TWI_WaitByteReceived(TWI1, RECV_TIMEOUT);
	v1 = TWI_ReadByte(TWI1);
	TWI_WaitTransferComplete(TWI1, RECV_TIMEOUT);
	
	c = (v2 << 8) | v1;
	
	return c;
}

int32_t read_register_24(uint8_t _i_address){
	int32_t c;
	uint8_t v1, v2, v3;
	
	TWI_StartRead(TWI1, ms5611_address, _i_address, 1);
	TWI_WaitByteReceived(TWI1, RECV_TIMEOUT);
	v3 = TWI_ReadByte(TWI1);
	TWI_WaitByteReceived(TWI1, RECV_TIMEOUT);
	v2 = TWI_ReadByte(TWI1);
	TWI_Stop(TWI1);
	TWI_WaitByteReceived(TWI1, RECV_TIMEOUT);
	v1 = TWI_ReadByte(TWI1);
	TWI_WaitTransferComplete(TWI1, RECV_TIMEOUT);
	
	c = ((int32_t)v3 << 16) | ((int32_t)v2 << 8) | (int32_t)v1;
	
	return c;
}

void set_OSR(ms5611_osr_t osr)
{
	switch (osr)
	{
		case MS5611_OSR_256:
		conversion_time = 1100;    //0.60ms
		break;
		case MS5611_OSR_512:
		conversion_time = 1600;    //1.17ms
		break;
		case MS5611_OSR_1024:
		conversion_time = 2700;    //2.28ms
		break;
		case MS5611_OSR_2048:
		conversion_time = 5000;    //4.54ms
		break;
		case MS5611_OSR_4096:
		conversion_time = 9500;    //9.04ms
		break;
	}

	OSR = osr;
}

void reset(void){
	TWI_StartWrite(TWI1, ms5611_address, 0, 0, MS5611_CMD_RESET);
	TWI_Stop(TWI1);
	TWI_WaitByteSent(TWI1, XMIT_TIMEOUT);
	TWI_WaitTransferComplete(TWI1, XMIT_TIMEOUT);
}

void readPROM(void) {
	
	C1 = read_register_16(MS5611_CMD_READ_PROM + (0 * MS5611_PROM_REG_SIZE));
	C2 = read_register_16(MS5611_CMD_READ_PROM + (1 * MS5611_PROM_REG_SIZE));
	C3 = read_register_16(MS5611_CMD_READ_PROM + (2 * MS5611_PROM_REG_SIZE));
	C4 = read_register_16(MS5611_CMD_READ_PROM + (3 * MS5611_PROM_REG_SIZE));
	C5 = read_register_16(MS5611_CMD_READ_PROM + (4 * MS5611_PROM_REG_SIZE));
	C6 = read_register_16(MS5611_CMD_READ_PROM + (5 * MS5611_PROM_REG_SIZE));
}

void start_conv_D1(){
	TWI_StartWrite(TWI1, ms5611_address, 0, 0, MS5611_CMD_CONV_D1 + OSR);
	TWI_Stop(TWI1);
	TWI_WaitByteSent(TWI1, XMIT_TIMEOUT);
	TWI_WaitTransferComplete(TWI1, XMIT_TIMEOUT);
}

void start_conv_D2(){
	TWI_StartWrite(TWI1, ms5611_address, 0, 0, MS5611_CMD_CONV_D2 + OSR);
	TWI_Stop(TWI1);
	TWI_WaitByteSent(TWI1, XMIT_TIMEOUT);
	TWI_WaitTransferComplete(TWI1, XMIT_TIMEOUT);
}

uint32_t get_raw_pressure(){	
	return read_register_24(MS5611_CMD_ADC_READ);
}

uint32_t get_raw_temperature(){	
	return read_register_24(MS5611_CMD_ADC_READ);
}

void init_ms5611(ms5611_osr_t osr){
	reset();
	set_OSR(osr);
	_delay_ms(100);
	readPROM();
}

void temp_comp_II(void){
	if(TEMP<2000){
		//Second order temperature compensation
		int32_t T2 = dT*dT/(1<<31);
		int64_t OFF2 = 2.5*(TEMP-2000)*(TEMP-2000);
		int64_t SENS2 = 1.25*(TEMP-2000)*(TEMP-2000);
		TEMP -= T2;
		OFF -= OFF2;
		SENS -= SENS2;
	}
}

void calculate_compensated_pressure(){
	//D1 & D2 should be known
	//PROM should have been read
	
	start_conv_D1();
	_delay_us(conversion_time);
	D1 = get_raw_pressure();
	
	start_conv_D2();
	_delay_us(conversion_time);
	D2 = get_raw_temperature();
	
	//calculate temperature
	dT = D2 - C5*(1<<8);
	TEMP = 2000 + dT*C6/(1<<23); 
	
	//calculate temperature compensated pressure
	OFF = C2*(1<<16) + (C4*dT)/(1<<7); 
	SENS = C1*(1<<15) + (C3*dT)/(1<<8);
		
	////////// SECOND ORDER TEMPERATURE COMPENSATION ///////////
	if (do_second_order_temp_compensation) temp_comp_II();
	
	P = (D1*SENS/(1<<21) - OFF)/(1<<15);
}

void calculate_altitude(void){
	
	//R = 8.3144598 J/mol/K
	//g = 9.80665 m/s2
	//M = 0.0289644 kg/mol
	//R/M/g = 29.2717593329
	//T_k = 273.15 K
	//TEMP = 100*(reading in degree celcius scale)
	//P is in Pascals
	
	if(alt_ground_set) alt = -29.2717593329 * (TEMP/100 + 273.15) * log(P) - alt_ground;
	else alt = -29.2717593329 * (TEMP/100 + 273.15) * log(P);
}

void calculate_ground_level(void){
	calculate_altitude();
	alt_ground = alt;
	alt_ground_set = true;
}

void print_ms5611_data(void){
	calculate_compensated_pressure();
	calculate_altitude();
	write("Pressure: "); print(P);
	write("  Temperature: "); println(TEMP);
	write("Altitude: "); println(alt);
}

void init_altitude(void){
	calculate_compensated_pressure();
	calculate_ground_level();
}

#endif
