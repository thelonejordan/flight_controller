
#include <asf.h>

void waitup(int num)
{
	volatile int i, j;
	for(i=0; i<num; i++)
	{
		j++;
	}
}

int main (void)
{
	/* Insert system clock initialization code here (sysclk_init()). */
	
    EFC0->EEFC_FMR = EEFC_FMR_FWS(4);
    EFC1->EEFC_FMR = EEFC_FMR_FWS(4);

	pmc_switch_mck_to_pllack(PMC_MCKR_PRES_CLK_2);// core clock 84MHz
	
	board_init();
	
	PIOB->PIO_PER |= PIO_PER_P27;   //PIO Enable Register (not required though)
	PIOB->PIO_OER |= PIO_OER_P27 ; //Output Enable Register
	

	/* Insert application code here, after the board has been initialized. */
	while (1)
	{
		//led is on
		PIOB->PIO_SODR |= PIO_SODR_P27; //Set Output Data Register
		//wait
		waitup(1000000);
		//led is off
		PIOB-> PIO_CODR |= PIO_CODR_P27; //Clear Output Data Register
		//wait
		waitup(1000000);
	}
}
