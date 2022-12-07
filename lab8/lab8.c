// lab8.c
// Runs on TM4C123G Launchpad

/*	This program is intended to operate using the TM4c123G Launchpad
 *	in combination with the stepper module with external power
 *	and a unipolar stepper motor.  The motor is driven using a
 *	half-step mode.
 *
 *	The motor speed is fixed and determined by a speed variable
 *	that is initialized in the program.  The motor moves forward until a limit
 *	switch is depressed, then moves 180 degrees back from its current position
 *
 *	It uses  direct register access to implement the functionality.
 *
 *	This program uses an 80MHz sysclock and the Precison Internal
 *	oscillator to drive the SysTick timer.
 *
 *	PF2 is used to indicate that the program is executing - the blue LED blinks
 *
 *	The program displays a running count on the upper 4 bits of
 *	the stepper motor module.  The count function and the blinking
 *	blue LED are just for indication of execution and can be removed
 *	with no effects on the motor operation.
 *
 *	The stepper motor phases are driven using PD0..PD3
 *	A count is displayed using PE0..PE3
 *	The analog input is AIN9 (PE4)
 *
 *	Written by:  Keith Bova, University of New Mexico, 12/7/22
 */

//****************************************************************
#include <stdint.h>
#include <stdbool.h>
#include <tm4c123gh6pm.h>

#define DELAY_VALUE 0xF9F			// 0xF9F = 1 mSec delay at 4 MHz
#define SYSDIV2 4

int speed = 20;					// delay value for stepper motor steps

/* ************************* Prototypes ****************************** */
void SysTick_init(void);
void SysTick_mSecDelay(uint32_t delay);

void moveMotorOneStepInForwardDirection()
{
// seguence for rotating the stepper motor in half step mode
		// in the forward direction
		GPIO_PORTD_DATA_R =  (GPIO_PORTD_DATA_R & 0xF0) + 0x8;  // step 1
		SysTick_mSecDelay(speed);
		GPIO_PORTD_DATA_R =  (GPIO_PORTD_DATA_R & 0xF0) + 0xC;  // step 2
		SysTick_mSecDelay(speed);
		GPIO_PORTD_DATA_R =  (GPIO_PORTD_DATA_R & 0xF0) + 0x4;  // step 3
		SysTick_mSecDelay(speed);
		GPIO_PORTD_DATA_R =  (GPIO_PORTD_DATA_R & 0xF0) + 0x6;  // step 4
		SysTick_mSecDelay(speed);
		GPIO_PORTD_DATA_R =  (GPIO_PORTD_DATA_R & 0xF0) + 0x2;  // step 5
		SysTick_mSecDelay(speed);
		GPIO_PORTD_DATA_R =  (GPIO_PORTD_DATA_R & 0xF0) + 0x3;  // step 6
		SysTick_mSecDelay(speed);
		GPIO_PORTD_DATA_R =  (GPIO_PORTD_DATA_R & 0xF0) + 0x1;  // step 7
		SysTick_mSecDelay(speed);
		GPIO_PORTD_DATA_R =  (GPIO_PORTD_DATA_R & 0xF0) + 0x9;  // step 8
		SysTick_mSecDelay(speed);

}


void moveMotorOneStepInReverseDirection()
{
// seguence for rotating the stepper motor in half step mode
		// in the reverse direction
		GPIO_PORTD_DATA_R =  (GPIO_PORTD_DATA_R & 0xF0) + 0x8;  // step 1
		SysTick_mSecDelay(speed);
		GPIO_PORTD_DATA_R =  (GPIO_PORTD_DATA_R & 0xF0) + 0x9;  // step 8
		SysTick_mSecDelay(speed);
		GPIO_PORTD_DATA_R =  (GPIO_PORTD_DATA_R & 0xF0) + 0x1;  // step 7
		SysTick_mSecDelay(speed);
		GPIO_PORTD_DATA_R =  (GPIO_PORTD_DATA_R & 0xF0) + 0x3;  // step 6
		SysTick_mSecDelay(speed);
		GPIO_PORTD_DATA_R =  (GPIO_PORTD_DATA_R & 0xF0) + 0x2;  // step 5
		SysTick_mSecDelay(speed);
		GPIO_PORTD_DATA_R =  (GPIO_PORTD_DATA_R & 0xF0) + 0x6;  // step 4
		SysTick_mSecDelay(speed);
		GPIO_PORTD_DATA_R =  (GPIO_PORTD_DATA_R & 0xF0) + 0x4;  // step 3
		SysTick_mSecDelay(speed);
		GPIO_PORTD_DATA_R =  (GPIO_PORTD_DATA_R & 0xF0) + 0xC;  // step 2
		SysTick_mSecDelay(speed);


}

int main(void){
	int count = 0;						// used as a value to drive the external LEDs

	_Bool moveMotorInForwardDirection, moveMotorInReverseDirection;

	int input;							// state of input switches 0x11 - neither pressed

	/* ------------ configure system clock for 80 Mhz operation -------------*/
    SYSCTL_RCC2_R |= 0x80000000;        // Use RCC2
    SYSCTL_RCC2_R |= 0x00000800;        // Bypass PLL while initializing it
                                        // Select crystal value and osc source
    SYSCTL_RCC_R = (SYSCTL_RCC_R & ~0x000007C0)     // clear bits 10-6
                + 0x00000540;           // 10101 configure for 16Mhz XTL
    SYSCTL_RCC2_R &= ~0x00000070;       //  Use main oscillator
    SYSCTL_RCC2_R &= ~0x00002000;       //  Activate PLL - clear PWRDN
    SYSCTL_RCC2_R |= 0x40000000;        //  Set system divider
    SYSCTL_RCC2_R = (SYSCTL_RCC2_R & ~0x1FC00000) +(SYSDIV2<<22);
    while((SYSCTL_RIS_R&0x00000040)==0){};  //  Wait for PLL to lock - poll PLLRIS
    SYSCTL_RCC2_R &= ~0x00000800;       //  Enable PLL by clearing BYPASS

	SysTick_init();             // initialize SysTick timer


/* ------  Initialize the GPIO ports -------	*/
	SYSCTL_RCGCGPIO_R |= 0x38;  // activate port D,E & F clocks
								// PORTF0 has special function, need to unlock to modify
    GPIO_PORTF_LOCK_R = 0x4C4F434B;	// unlock commit register
    GPIO_PORTF_CR_R |= 0x01;		// make PORTF0 configurable
    GPIO_PORTF_LOCK_R = 0;			// lock commit register
									// configure PORTF for switch input and LED output
    GPIO_PORTF_DIR_R &= ~0x11;		// make PORTF 4, 0 input for switch
    GPIO_PORTF_DIR_R |= 0x04;		// make PORTF 2 output for blue LED
	GPIO_PORTE_DIR_R |= 0x0F;		// make PE0..PE3 out
	GPIO_PORTD_DIR_R |= 0x0F;		// make PD0..PD3 out
	GPIO_PORTF_AFSEL_R &= ~0x15;	// disable alt funct on PF4,PF2,PF0
	GPIO_PORTE_AFSEL_R &= ~0x0F;	// disable alt funct on PE0..PE3
	GPIO_PORTD_AFSEL_R &= ~0x0F;	// disable alt funct on PD0..PD3
	GPIO_PORTF_DEN_R |= 0x15;		// enable digital I/O on on PF4,PF2,PF0
	GPIO_PORTE_DEN_R |= 0x0F;		// enable digital I/O on PE0..PE3
	GPIO_PORTD_DEN_R |= 0x0F;		// enable digital I/O on PD0..PD3
	GPIO_PORTF_PUR_R |= 0x11;		// enable pull up for PORTF4, 0

								// configure PF4,PF2,PF0 as GPIO (Selectively)
	GPIO_PORTF_PCTL_R = (GPIO_PORTF_PCTL_R&0xFFF0F0F0)+0x00000000;
								// configure PE0..PE3 as GPIO (Selectively)
	GPIO_PORTE_PCTL_R = (GPIO_PORTF_PCTL_R&0xFFFF0000)+0x00000000;
								// configure PD0..PD3 as GPIO (Selectively)
	GPIO_PORTD_PCTL_R = (GPIO_PORTD_PCTL_R&0xFFFF0000)+0x00000000;
	GPIO_PORTF_AMSEL_R = 0;     // disable analog functionality on PF
	GPIO_PORTE_AMSEL_R = 0;     // disable analog functionality on PE
	GPIO_PORTD_AMSEL_R = 0;     // disable analog functionality on PD

		moveMotorInForwardDirection = true;
		moveMotorInReverseDirection = false;
		int numberOfStepsThatEquals180DegreesOnMotor = 100;

  while(1){
		input = GPIO_PORTF_DATA_R & 0x11;	// raw input bits - others masked
		SysTick_mSecDelay(10);

	if (moveMotorInForwardDirection)
	{
			moveMotorOneStepInForwardDirection();
	}

	if (input == 1 )
	{
		moveMotorInReverseDirection = true;
		moveMotorInForwardDirection = false;
		for(int i = 0; i < numberOfStepsThatEquals180DegreesOnMotor; i++)
		{
			moveMotorOneStepInReverseDirection();
		}
	}

	GPIO_PORTE_DATA_R = count & 0xF;          // lower 4 count bits - PE0..PE3 - diagnostic



	 GPIO_PORTF_DATA_R = GPIO_PORTF_DATA_R^0x04; // toggle PF2 - Diagnostic

    count++;							// Diagnostic
		count = count % 256;				// make count modulo 256

	}
}

void SysTick_init(void){
	NVIC_ST_CTRL_R = 0;					// 1. disable SysTick before configuring
	NVIC_ST_RELOAD_R = DELAY_VALUE;		// 2. set to desired delay value (1 mSec)
	NVIC_ST_CURRENT_R = 0;				// 3. clear CURRENT by writing any value
	NVIC_ST_CTRL_R &= ~0x00000004;		// 4. set clock to POSC/4
	NVIC_ST_CTRL_R |= 0x00000001;		// 5. enable SysTick timer
}

/* 1 mSec Time delay using busy wait.	*/
void SysTick_mSecDelay(uint32_t delay){
  uint32_t i;
  for(i=0; i<delay; i++){
    NVIC_ST_RELOAD_R = DELAY_VALUE;		// wait one cycle = DELAY_VALUE
    NVIC_ST_CURRENT_R = 0;

    while((NVIC_ST_CTRL_R & 0x00010000) == 0){  };
  }
}
