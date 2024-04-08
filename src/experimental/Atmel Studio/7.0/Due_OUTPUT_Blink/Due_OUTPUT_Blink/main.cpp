
#include "sam3x8e.h"

void waitup(int num)
{
	volatile int i, j;
	for(i=0; i<num; i++)
	{
		j++;
	}
}

int main(void)
{
	SystemInit();
	//Declare pin 13 as output
	PIOB->PIO_PER |= PIO_PER_P27;              //PIO Enable Register (not required though)
	PIOB->PIO_OER |= PIO_OER_P27 ;             //Output Enable Register
	
	/*while (1)
	{
		//led is on
		PIOB->PIO_SODR |= PIO_SODR_P27;        //Set Output Data Register
		waitup(1000000);                       //wait
		//led is off
		PIOB-> PIO_CODR |= PIO_CODR_P27;       //Clear Output Data Register
		waitup(1000000);                       //wait
	}*/
	
	while (1)
	{
		waitup(1000000);
		if((PIOB->PIO_ODSR & PIO_ODSR_P27) == PIO_ODSR_P27) PIOB->PIO_CODR |= PIO_CODR_P27;
		else PIOB->PIO_SODR |= PIO_SODR_P27;
	}
}

