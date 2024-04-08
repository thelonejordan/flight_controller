/*
 * Run_motors.cpp
 *
 * Created: 24-06-2017 19:07:03
 * Author : Jyotirmaya Mahanta
 */ 


#include "sam3x8e.h"
#include "InitTimer.h"
#include "InitUART.h"

int convert_receiver_channel(uint8_t function);
uint32_t cpins = (1u<<13)|(1u<<15)|(1u<<17);
uint32_t bpins = (1u<<21);
uint32_t cstat, bstat;

uint8_t last_channel[7];
volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, loop_counter;
int esc_1, esc_2, esc_3, esc_4;
int receiver_input[7];

uint32_t timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
uint32_t timer[7], current_time_1, current_time_2;
uint32_t loop_timer;


int main(void)
{
	SystemInit();
	TIMERbegin();
	UARTbegin(57600);
	
	PMC->PMC_PCER0 |= (1u<<ID_PIOA)|(1u<<ID_PIOC);        //enable peripheral clock
	
	//  1  2  3  4  5   6
	//  35 37 39 41 43  45
	//  C3 C5 C7 C9 A20 C18
	PIOA->PIO_IER |= PIO_IER_P20;                                                   //enable pin change interrupts
	PIOC->PIO_IER |= PIO_IER_P3|PIO_IER_P5|PIO_IER_P7|PIO_IER_P9|PIO_IER_P18;       //enable pin change interrupts
	
	PIOA->PIO_ODR |= PIO_ODR_P20;                                                   //enable input
	PIOC->PIO_ODR |= PIO_ODR_P3|PIO_ODR_P5|PIO_ODR_P7|PIO_ODR_P9|PIO_ODR_P18;       //enable input
	
	//46 48 50 52 - ESC channels - c17 c15 c13 b21
	PIOC->PIO_OER |= PIO_OER_P13|PIO_OER_P15|PIO_OER_P17;
	PIOB->PIO_OER |= PIO_OER_P21;
	
	//NVIC_EnableIRQ(PIOA_IRQn);              //configure NVIC
	NVIC_EnableIRQ(PIOC_IRQn);              //configure NVIC
	
	loop_timer = micros();
	
	while (1)
	{
		while((micros() - loop_timer) < 4000);
		loop_timer = micros();
		PIOC->PIO_SODR |= cpins;
		PIOB->PIO_SODR |= bpins;
		receiver_input_channel_1 = convert_receiver_channel(3);
		timer_channel_1 = loop_timer + receiver_input_channel_1;
		timer_channel_2 = loop_timer + receiver_input_channel_1;
		timer_channel_3 = loop_timer + receiver_input_channel_1;
		timer_channel_4 = loop_timer + receiver_input_channel_1;
	
		while(((cstat & cpins) != 0) && ((bstat & bpins) !=0)){
			esc_loop_timer = micros();
			if(esc_loop_timer>= timer_channel_1) PIOC->PIO_CODR |= PIO_CODR_P13;
			if(esc_loop_timer>= timer_channel_1) PIOC->PIO_CODR |=PIO_CODR_P15;
			if(esc_loop_timer>= timer_channel_1) PIOC->PIO_CODR |=PIO_CODR_P17;
			if(esc_loop_timer>= timer_channel_1) PIOB->PIO_CODR |=PIO_CODR_P21;
			cstat = PIOC->PIO_PDSR;
			bstat = PIOB->PIO_PDSR;
		}
				
	}
}

void PIOC_Handler(){
	current_time_1 = micros();
	
	//channel 1
	if ((PIOC->PIO_PDSR & PIO_PDSR_P3) == PIO_PDSR_P3){
		if(last_channel[1] == 0){
			last_channel[1] = 1;
			timer[1] = current_time_1;
		}
	}
	
	else if (last_channel[1] == 1){
		last_channel[1] = 0;
		receiver_input[1] = micros() - timer[1];
	}
	
	//channel 2
	if ((PIOC->PIO_PDSR & PIO_PDSR_P5) == PIO_PDSR_P5){
		if(last_channel[2] == 0){
			last_channel[2] = 1;
			timer[2] = current_time_1;
		}
	}
	
	else if (last_channel[2] == 1){
		last_channel[2] = 0;
		receiver_input[2] = micros() - timer[2];
	}
	
	//channel 3
	if ((PIOC->PIO_PDSR & PIO_PDSR_P7) == PIO_PDSR_P7){
		if(last_channel[3] == 0){
			last_channel[3] = 1;
			timer[3] = current_time_1;
		}
	}
	
	else if (last_channel[3] == 1){
		last_channel[3] = 0;
		receiver_input[3] = micros() - timer[3];
		println((float)receiver_input[3]);
	}
	
	//channel 4
	if ((PIOC->PIO_PDSR & PIO_PDSR_P9) == PIO_PDSR_P9){
		if(last_channel[4] == 0){
			last_channel[4] = 1;
			timer[4] = current_time_1;
		}
	}
	
	else if (last_channel[4] == 1){
		last_channel[4] = 0;
		receiver_input[4] = micros() - timer[4];
	}
	
	//channel 6
	if ((PIOC->PIO_PDSR & PIO_PDSR_P18) == PIO_PDSR_P18){
		if(last_channel[6] == 0){
			last_channel[6] = 1;
			timer[6] = current_time_1;
		}
	}
	
	else if (last_channel[6] == 1){
		last_channel[6] = 0;
		receiver_input[6] = micros() - timer[6];
	}
		
	PIOC->PIO_ISR;
}

void PIOA_Handler(){
	current_time_2 = micros();
	
	//channel 5
	if ((PIOA->PIO_PDSR & PIO_PDSR_P20) == PIO_PDSR_P20){
		if(last_channel[5] == 0){
			last_channel[5] = 1;
			timer[5] = current_time_2;
		}
	}
	
	else if (last_channel[5] == 1){
		last_channel[5] = 0;
		receiver_input[5] = micros() - timer[5];
	}
	
	PIOA->PIO_ISR;
}

int convert_receiver_channel(uint8_t function){
	int low=0, center=0, high=0, actual=0;
	int difference=0;

	//roll
	if(function==1){
		low = 1009;
		center = 1512;
		high = 2004;
	}
	//pitch
	else if(function==2){
		low = 1022;
		center = 1526;
		high = 2015;
	}
	//throttle
	else if(function==3){
		low = 1036;
		center = 1540;
		high = 2029;
	}
	//yaw
	else if(function==4){
		low = 1050;
		center = 1554;
		high = 2043;
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