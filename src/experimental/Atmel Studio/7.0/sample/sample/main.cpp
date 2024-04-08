
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

uint32_t all_pins = (1u<<1)|(1u<<3)|(1u<<5)|(1u<<7)|(1u<<8)|(1u<<9);
uint32_t esc_pins = (1u<<13)|(1u<<15)|(1u<<17)|(1u<<19);

int main(void)
{
    SystemInit();
	UARTbegin(57600);
	TIMERbegin();
	setup_interrupts();
	
	
    while (1) {
		println(receiver_input[4]);
		_delay_ms(4);
    }
}


void PIOC_Handler(){
	current_time = micros();
	status = PIOC->PIO_PDSR;
	
	//channel 4 ====================================================================================
	if (status & (1u<<5)){
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