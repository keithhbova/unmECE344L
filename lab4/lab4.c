#include <tm4c123gh6pm.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
		
#define CONSTANT_FOR_SYS_TICK_AT_A_QUARTER_SECOND 0xf423f		

#define SYSDIV2 2


void turnOnGreenBlueLight()
{
	GPIO_PORTF_DATA_R = 0x0C;			
	
}

void turnOnRedGreenBlueLight()
{
	GPIO_PORTF_DATA_R = 0x0E;						
}


void turnOffLightsOnTheBoard()
{
	GPIO_PORTF_DATA_R = 0;							// lights out!
}

void turnOnBlueLight()
{
	GPIO_PORTF_DATA_R = 4;			//light blue
}

void turnOnRedLight()
{
	GPIO_PORTF_DATA_R = 2;      // light red
}

void turnOnGreenLight()
{
	GPIO_PORTF_DATA_R = 8;      // light green
}

void initializeLightsOnBoard()
{
	SYSCTL_RCGCGPIO_R |= 0x00000020; // (a) activate clock for port F
       
    /* PORTF0 has special function, need to unlock to modify */
    GPIO_PORTF_LOCK_R = 0x4C4F434B;   /* unlock commit register */
    GPIO_PORTF_CR_R = 0x01;           /* make PORTF0 configurable */
    GPIO_PORTF_LOCK_R = 0;            /* lock commit register */
    /* configure PORTF for switch input and LED output */
    GPIO_PORTF_DIR_R &= ~0x11;         /* make PORTF 4, 0 input for switch */
    GPIO_PORTF_DIR_R |= 0x0E;          /* make PORTF3, 2, 1 output for LEDs */
GPIO_PORTF_AFSEL_R &= ~0x11;     // disable alt funct on PF4, PF0
    GPIO_PORTF_DEN_R |= 0x1F;          /* make PORTF4-0 digital pins */
    GPIO_PORTF_PUR_R |= 0x11;          /* enable pull up for PORTF4, 0 */
    GPIO_PORTF_DATA_R = 0;
	GPIO_PORTF_DIR_R |= 0x0E;         	/* make PORTF3, 2, 1 output for LEDs */
}


#if 0
void delayUsingTimer0InMs(const uint32_t timeToDelayInMs)
{
	SYSCTL_RCGCTIMER_R |= 1;     		// enable clock to Timer Block 0
    TIMER0_CTL_R = 0;            		// disable Timer before initialization
    TIMER0_CFG_R = 0x00;         		// 32-bit option
    TIMER0_TAMR_R = 0x01;        		// one-shot mode and down-counter
    TIMER0_TAILR_R = 80000 * timeToDelayInMs - 1;  // Timer A interval load value register
    TIMER0_ICR_R = 0x1;          		// clear the TimerA timeout flag
    TIMER0_CTL_R |= 0x01;        		// enable Timer A after initialization	
    while ((TIMER0_RIS_R & 0x1) == 0) ;  // wait for TimerA timeout flag to set
}
#endif

void initializeSysTick(void){
	NVIC_ST_CTRL_R = 0;								// 1. disable SysTick before configuring
	NVIC_ST_RELOAD_R = CONSTANT_FOR_SYS_TICK_AT_A_QUARTER_SECOND;		// 2. set to desired delay value
	NVIC_ST_CURRENT_R = 0;						// 3.	clear CURRENT by writing any value
	NVIC_ST_CTRL_R &= ~0x00000004;		// 4. set clock to POSC/4
	NVIC_ST_CTRL_R |= 0x00000001;			// 5. enable SysTick timer
}

void SysTick_delay(void){
	NVIC_ST_CURRENT_R = 0;										// 1.	clear CURRENT by writing any value
	while((NVIC_ST_CTRL_R & 0x00010000)==0)		// 2. wait for count flag to be set
		{
		}
	}


void initializeSystemClockFor80mhz()
{

    SYSCTL_RCC2_R |= 0x80000000;
/*  Bypass the PLL while initializing it */
    SYSCTL_RCC2_R |= 0x00000800;
/*  Select the crystal value and oscillator source */
    SYSCTL_RCC_R = (SYSCTL_RCC_R & ~0x000007C0)     // clear bits 10-6
                + 0x00000540;       // 10101 configure for 16Mhz XTL
/*  Set for main oscillator source  */
    SYSCTL_RCC2_R &= ~0x00000070;
/*  Activate the PLL by clearing PWRDN  */
    SYSCTL_RCC2_R &= ~0x00002000;
/*  Set the desired system divider  */
    SYSCTL_RCC2_R |= 0x40000000;
    SYSCTL_RCC2_R = (SYSCTL_RCC2_R & ~0x1FC00000) +(SYSDIV2<<22);
/*  Wait for the PLL to lock by polling PLLRIS  */
    while((SYSCTL_RIS_R&0x00000040)==0){};
/*  Enable use of the PLL by clearing BYPASS  */
    SYSCTL_RCC2_R &= ~0x00000800;
	
}


void initializeGPIOPorts()
{
	SYSCTL_RCGCGPIO_R |= 0x38;  // activate port D,E & F clocks
	
  GPIO_PORTF_DIR_R |= 0x04;   // make PF2 out (built-in blue LED)
	GPIO_PORTE_DIR_R |= 0x0F;   // make PE0..PE3 out
  GPIO_PORTD_DIR_R |= 0x0F;   // make PD0..PD3 out 
	
  GPIO_PORTF_AFSEL_R &= ~0x04;// disable alt funct on PF2
	GPIO_PORTE_AFSEL_R &= ~0x0F;// disable alt funct on PE0..PE3
  GPIO_PORTD_AFSEL_R &= ~0x0F;// disable alt funct on PD0..PD3
	
  GPIO_PORTF_DEN_R |= 0x04;   // enable digital I/O on PF2
	GPIO_PORTE_DEN_R |= 0x0F;   // enable digital I/O on PE0..PE3
  GPIO_PORTD_DEN_R |= 0x0F;   // enable digital I/O on PD0..PD3
	
                              // configure PF2 as GPIO (Selectively - others left unchanged)
  GPIO_PORTF_PCTL_R = (GPIO_PORTF_PCTL_R&0xFFFFF0FF)+0x00000000;
												// configure PE0..PE3 as GPIO (Selectively - others left unchanged)
	GPIO_PORTE_PCTL_R = (GPIO_PORTF_PCTL_R&0xFFFF0000)+0x00000000;
                        // configure PD0..PD3 as GPIO (Selectively - others left unchanged)
  GPIO_PORTD_PCTL_R = (GPIO_PORTD_PCTL_R&0xFFFF0000)+0x00000000;
	
  GPIO_PORTF_AMSEL_R = 0;     // disable analog functionality on PF 
	GPIO_PORTE_AMSEL_R = 0;     // disable analog functionality on PE
  GPIO_PORTD_AMSEL_R = 0;     // disable analog functionality on PD
	
}

void displayDecimalNumberOnGpioBoardInBinary(uint8_t decimalToDisplayInBinary)
{
	const uint32_t decimalToDisplayInBinaryMod256 = decimalToDisplayInBinary % 256;
	GPIO_PORTE_DATA_R = decimalToDisplayInBinary & 0xF;          // lower 4 count bits - PE0..PE3
	GPIO_PORTD_DATA_R = (decimalToDisplayInBinary & 0xF0) >> 4;  // upper 4 count bits - PD0..PD3
}

void turnOnLightsBasedOffCurrentState(const uint8_t currentState)
{
	turnOffLightsOnTheBoard();
	
	if(0 == currentState)
	{
		turnOnRedGreenBlueLight();
	}
	
	if(4 == currentState)
	{
		turnOnBlueLight();
	}
	
	if(8 == currentState)
	{
		turnOnGreenBlueLight();
	}
	
	if(12 == currentState)
	{
		turnOnBlueLight();
	}
	if(16 == currentState)
	{
		turnOnBlueLight();	
	}
	
}


#if 1

int main()
{
	initializeSystemClockFor80mhz();
	initializeSysTick();
	initializeGPIOPorts();
	initializeLightsOnBoard();
	
	uint32_t currentState;
	uint8_t	count;
	currentState = count = 0;
	
	while(true)
	{
		turnOnLightsBasedOffCurrentState(currentState);
		displayDecimalNumberOnGpioBoardInBinary(count);
		
		currentState++;
		if(15 < currentState)
		{
			currentState = 0;
		}
		count++;

		SysTick_delay();
	}	
	
	return 0;
}
#endif 



#if 0
int main()
{
	initializeSystemClockFor80mhz();
	//initializeSysTick();
	initializeGPIOPorts();
	initializeLightsOnBoard();
	
	uint32_t currentState;
	uint8_t	count;
	currentState = count = 0;
	
	while(true)
	{
		turnOnLightsBasedOffCurrentState(currentState);
		displayDecimalNumberOnGpioBoardInBinary(count);
		
		currentState++;
		if(15 < currentState)
		{
			currentState = 0;
		}
		count++;

		delayUsingTimer0InMs(250);
	}	
	
	return 0;
}

#endif 