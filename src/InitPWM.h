#ifndef InitPWM
#define InitPWM

#include "sam3x8e.h"

// C PWML peripheral B
//due:  09 08 07 06
//pins: 21 22 23 24
//PWMl: 04 05 06 07

#define esc_1_pin 4
#define esc_2_pin 5
#define esc_3_pin 6
#define esc_4_pin 7

#define esc_pins (0x0F<<21)    //0b1111<<21
#define CHID     (0x0F<<4)     //0b1111<<4

void PWMbegin(){
	PMC->PMC_PCER1 |= PMC_PCER1_PID36;								 // Enable PWM
	
	PWM->PWM_DIS = CHID;
	PWM->PWM_SCM |= PWM_SCM_SYNC4|PWM_SCM_SYNC5|PWM_SCM_SYNC6|PWM_SCM_SYNC7;
	PWM->PWM_SCM &= ~(1u<<16);
	PIOC->PIO_PDR |= esc_pins;										 // Set PWM pin to an output
	PIOC->PIO_ABSR |= esc_pins;                                      // Set PWM pin peripheral type A or B, in this case B
	PWM->PWM_CLK = PWM_CLK_PREA(0)| PWM_CLK_DIVA(42);				 // Set the PWM clock rate to 2MHz (84MHz/42)

	PWM->PWM_CH_NUM[0].PWM_CMR = PWM_CMR_CALG | PWM_CMR_CPRE_CLKA;   // Enable dual slope PWM and set the clock source as CLKA
	PWM->PWM_CH_NUM[0].PWM_CPRD = 2500;							     // Set the PWM frequency 2MHz/(2 * 2500) = 400Hz
	PWM->PWM_CH_NUM[0].PWM_CDTY = PWM_CDTY_CDTY(2000);               // Set the PWM high period to 1000us

	PWM->PWM_CH_NUM[4].PWM_CMR = PWM_CMR_CALG | PWM_CMR_CPRE_CLKA;   // Enable dual slope PWM and set the clock source as CLKA
	PWM->PWM_CH_NUM[4].PWM_CPRD = 2500;							     // Set the PWM frequency 2MHz/(2 * 2500) = 400Hz
	PWM->PWM_CH_NUM[4].PWM_CDTY = PWM_CDTY_CDTY(2000);               // Set the PWM high period to 1000us
	
	PWM->PWM_CH_NUM[5].PWM_CMR = PWM_CMR_CALG | PWM_CMR_CPRE_CLKA;   // Enable dual slope PWM and set the clock source as CLKA
	PWM->PWM_CH_NUM[5].PWM_CPRD = 2500;							     // Set the PWM frequency 2MHz/(2 * 2500) = 400Hz
	PWM->PWM_CH_NUM[5].PWM_CDTY = PWM_CDTY_CDTY(2000);               // Set the PWM high period to 1000us
	
	PWM->PWM_CH_NUM[6].PWM_CMR = PWM_CMR_CALG | PWM_CMR_CPRE_CLKA;   // Enable dual slope PWM and set the clock source as CLKA
	PWM->PWM_CH_NUM[6].PWM_CPRD = 2500;							     // Set the PWM frequency 2MHz/(2 * 2500) = 400Hz
	PWM->PWM_CH_NUM[6].PWM_CDTY = PWM_CDTY_CDTY(2000);               // Set the PWM high period to 1000us
	
	PWM->PWM_CH_NUM[7].PWM_CMR = PWM_CMR_CALG | PWM_CMR_CPRE_CLKA;   // Enable dual slope PWM and set the clock source as CLKA
	PWM->PWM_CH_NUM[7].PWM_CPRD = 2500;							     // Set the PWM frequency 2MHz/(2 * 2500) = 400Hz
	PWM->PWM_CH_NUM[7].PWM_CDTY = PWM_CDTY_CDTY(2000);               // Set the PWM high period to 1000us
	
	PWM->PWM_ENA |= PWM_ENA_CHID0;                                             // Enable the PWM channels
}

void set_pulse(uint16_t pulse_1,uint16_t pulse_2, uint16_t pulse_3, uint16_t pulse_4){
	
	/*Sequence for Method 1:
	1. Select the manual write of duty-cycle values and the manual update by setting the UPDM field to 0 in the
	PWM_SCM register
	2. Define the synchronous channels by the SYNCx bits in the PWM_SCM register.
	3. Enable the synchronous channels by writing CHID0 in the PWM_ENA register.
	4. If an update of the period value and/or the duty-cycle values and/or the dead-time values is required, write
	registers that need to be updated (PWM_CPRDUPDx, PWM_CDTYUPDx and PWM_DTUPDx).
	5. Set UPDULOCK to 1 in PWM_SCUC.
	6. The update of the registers will occur at the beginning of the next PWM period. At this moment the
	UPDULOCK bit is reset, go to Step 4.) for new values.
    */
	PWM->PWM_CH_NUM[esc_1_pin].PWM_CDTYUPD = pulse_1;
	PWM->PWM_CH_NUM[esc_2_pin].PWM_CDTYUPD = pulse_2;
	PWM->PWM_CH_NUM[esc_3_pin].PWM_CDTYUPD = pulse_3;
	PWM->PWM_CH_NUM[esc_4_pin].PWM_CDTYUPD = pulse_4;
	
	PWM->PWM_SCUC |= PWM_SCUC_UPDULOCK;
}

void set_all(uint16_t pulse){
	set_pulse(pulse, pulse, pulse, pulse);
}
#endif