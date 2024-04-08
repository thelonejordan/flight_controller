/*
 * NVIC.cpp
 *
 * Created: 24-06-2017 02:19:03
 * Author : Jyotirmaya Mahanta
 */ 


#include "sam3x8e.h"
#include "InitTimer.h"
#include "InitUART.h"

int convert_receiver_channel(uint8_t function);
void std_esc_calibrate(void);
void print_rx_raw(void);
void print_converted_rx(void);
void setup_interrupts(void);
void run_motors_1(void);
void run_motors_2(void);

volatile uint32_t current_time, status, odsr;
volatile uint32_t timer[7];
volatile uint8_t last_channel[7] = {0};
uint32_t esc_pin_stat;
volatile uint32_t receiver_input[7] = {0};
int rx_ch_1, rx_ch_2, rx_ch_3, rx_ch_4, rx_ch_5, rx_ch_6;
volatile uint32_t timer_1, timer_2, timer_3, timer_4;
uint32_t loop_timer, esc_loop_timer;
int throttle, start;

//  01 02 03 04 05 06
//  33 35 37 39 41 40
//  C1 C3 C5 C7 C9 C8

uint32_t all_pins = (1u<<1)|(1u<<3)|(1u<<5)|(1u<<7)|(1u<<8)|(1u<<9);
uint32_t esc_pins = (1u<<13)|(1u<<15)|(1u<<17)|(1u<<19);

int main(void)
{
    SystemInit();
    TIMERbegin();
	UARTbegin(57600);
	
	setup_interrupts();
	
	std_esc_calibrate();
	
	while(0){
		rx_ch_4 = convert_receiver_channel(4);
		println(receiver_input[4]);
		_delay_ms(3);
	}
	while (0){
		print_converted_rx();
	}
	
    while (0) {
		print_rx_raw();
    }
	
	while(0){
		run_motors_1();
	}
	
	loop_timer = micros();
	while(0){
		run_motors_2();
	}
	
	start = 0;
	while(0){
		//For starting the motors: throttle low and yaw left (step 1).
		print(rx_ch_3);
		write('\t');
		println(rx_ch_4);
		if((rx_ch_4 < 1050) && (rx_ch_3 < 1050))start = 1;
		//When yaw stick is back in the center position start the motors (step 2).
		if((start == 1) && (rx_ch_3 < 1050) && (rx_ch_4 > 1450)){
			start = 2;
		}
		
		//Stopping the motors: throttle low and yaw right.
		if(start == 2 && rx_ch_3 < 1050 && rx_ch_4 > 1950)start = 0;
		print(start);
		write('\t');
		rx_ch_3 = convert_receiver_channel(3);
		rx_ch_4 = convert_receiver_channel(4);
		_delay_ms(3);
	}
}

void PIOC_Handler(){
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

void std_esc_calibrate(void){
	//takes 24s in total
	int throttle;
	
	throttle = 2000;
	
	for(int i = 0; i<2500; i++){
		PIOC->PIO_SODR |= esc_pins;
		_delay_us(throttle);
		PIOC->PIO_CODR |= esc_pins;
		_delay_us(4000 - throttle);
	}
	
	throttle = 1000;
	
	for(int i = 0; i<2500; i++){
		
		PIOC->PIO_SODR |= esc_pins;
		_delay_us(throttle);
		PIOC->PIO_CODR |= esc_pins;
		_delay_us(4000 - throttle);
	}
	
	throttle = 1500;
	
	for(int i = 0; i<1000; i++){
		PIOC->PIO_SODR |= esc_pins;
		_delay_us(throttle);
		PIOC->PIO_CODR |= esc_pins;
		_delay_us(4000 - throttle);
	}
}

void print_rx_raw(void){
	print(receiver_input[1]);
	write('\t');
	print(receiver_input[2]);
	write('\t');
	print(receiver_input[3]);
	write('\t');
	print(receiver_input[4]);
	write('\t');
	print(receiver_input[5]);
	write('\t');
	println(receiver_input[6]);
	_delay_ms(4);
}

void print_converted_rx(void){
	
	rx_ch_1 = convert_receiver_channel(1);
	rx_ch_2 = convert_receiver_channel(2);
	rx_ch_3 = convert_receiver_channel(3);
	rx_ch_4 = convert_receiver_channel(4);
	print(rx_ch_1);
	write('\t');
	print(rx_ch_2);
	write('\t');
	print(rx_ch_3);
	write('\t');
	print(rx_ch_4);
	write('\t');
	print(rx_ch_5);
	write('\t');
	println(rx_ch_6);
	_delay_ms(4);
}

void setup_interrupts(void){
	PMC->PMC_PCER0 |= (1u<<ID_PIOC);        //enable peripheral clock
	PIOC->PIO_IER |= all_pins;              //enable pin change interrupts
	PIOC->PIO_PER |= all_pins;              //enable PIO controller
	PIOC->PIO_ODR |= all_pins;              //set as input
	NVIC_SetPriority(PIOC_IRQn, 0);
	NVIC_EnableIRQ(PIOC_IRQn);              //configure NVIC
	
	PIOC->PIO_PER |= esc_pins;
	PIOC->PIO_OER |= esc_pins;
}

void run_motors_1(void){
	rx_ch_3 = convert_receiver_channel(3);
	throttle = rx_ch_3;
	PIOC->PIO_SODR |= esc_pins;
	_delay_us(throttle);
	PIOC->PIO_CODR |= esc_pins;
	_delay_us(4000 - throttle);
}

void run_motors_2(void){
	while(micros() - loop_timer < 4000);
	loop_timer = micros();
	PIOC->PIO_SODR |= esc_pins;
	esc_pin_stat = PIOC->PIO_ODSR;
	rx_ch_3 = convert_receiver_channel(3);
	throttle = rx_ch_3;
	timer_1 = loop_timer + rx_ch_3;
	timer_2 = loop_timer + rx_ch_3;
	timer_3 = loop_timer + rx_ch_3;
	timer_4 = loop_timer + rx_ch_3;
	while(esc_pin_stat){
		esc_loop_timer = micros();
		if(esc_loop_timer>=timer_1)PIOC->PIO_CODR |= 1u<<13;
		if(esc_loop_timer>=timer_2)PIOC->PIO_CODR |= 1u<<15;
		if(esc_loop_timer>=timer_3)PIOC->PIO_CODR |= 1u<<17;
		if(esc_loop_timer>=timer_4)PIOC->PIO_CODR |= 1u<<19;
		esc_pin_stat = PIOC->PIO_ODSR & esc_pins;
	}
}