#ifndef RX
#define RX

//Receiver input variables
uint8_t last_channel[7];
volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
volatile int rec_1, rec_2, rec_3, rec_4, rec_5, rec_6;
volatile int receiver_input[7];
uint32_t timer[7], current_time;

//  01 02 03 04 05 06
//  33 35 37 39 41 40
//  C1 C3 C5 C7 C9 C8
const uint32_t rx_pins = (1u<<1)|(1u<<3)|(1u<<5)|(1u<<7)|(1u<<8)|(1u<<9);

uint32_t status;


void PIOC_Handler(void){
	
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

void setup_interrupts(void){
	PMC->PMC_PCER0 |= (1u<<ID_PIOC);        //enable peripheral clock
	PIOC->PIO_IER |= rx_pins;               //enable pin change interrupts
	PIOC->PIO_PER |= rx_pins;               //enable PIO controller
	PIOC->PIO_ODR |= rx_pins;               //set as input
	NVIC_SetPriority(PIOC_IRQn, 0);			//set priority
	NVIC_EnableIRQ(PIOC_IRQn);              //configure NVIC
}


#endif 