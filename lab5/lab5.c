/************************************************************************/ 
/* ECE 344L - Microprocessors – Fall 2022      */ 
/*            */ 
/*            */ 
/* lab5.c     */ 
/*            */ 
/*            */ 
/************************************************************************/ 
/* Author(s):  Keith Bova      */ 
/*                                          */ 
/************************************************************************/ 
/*   Detailed File Description:        */ 
/*  This program implements a basic cypher with uart & gpio  */ 
/*      */ 
/*            */ 
/************************************************************************/ 
/*  Revision History:  11/2/22  */ 
/*        */ 
/*            */ 
/************************************************************************/ 

#include <stdint.h>
#include <stdbool.h>					// needed for compatibility
#include <tm4c123gh6pm.h>


#define DELAY_VALUE 0x1E847F    		// 0x1E847F = 0.5 sec delay at 4MHz input clock
										// 0XF423F = 0.25 sec delay at 4MHz
#define SYSDIV2 4

#define SHIFT_NUMBER 2

/* ------- Prototypes --------  */
void UART2Tx(char c);
char UART2Rx(void);  
void SysTick_init(void);
void SysTick_delay(void);

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

char performShitfTransformationOnCapitalLetters(const char asciiCharToConvert)
{
	short shiftContainer = asciiCharToConvert -65;
	shiftContainer = shiftContainer + SHIFT_NUMBER;
	
	if(shiftContainer < 0)
	{
		shiftContainer = 26 + shiftContainer;
	}
	
	if(shiftContainer > 25)
	{
		shiftContainer = shiftContainer - 26;
	}
	
	return shiftContainer + 65;
}	
	

char performShitfTransformationOnLowercaseLetters(const char asciiCharToConvert)
{
	short shiftContainer = asciiCharToConvert - 97;
	shiftContainer = shiftContainer + SHIFT_NUMBER;
	
	if(shiftContainer < 0)
	{
		shiftContainer = 26 + shiftContainer;
	}
	
	if(shiftContainer > 25)
	{
		shiftContainer = shiftContainer - 26;
	}
	
	return shiftContainer + 97;
}	
	
int main(void)
{
    char c;
	initializeGPIOPorts();
	uint8_t numberOfKeystrokes = 0;
	/* Configure the system clock - SYSCLK = 80 MHz	*/
	
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

    while((SYSCTL_RIS_R&0x00000040)==0)
    {
    	
    };  //  Wait for PLL to lock - poll PLLRIS
    
    
    SYSCTL_RCC2_R &= ~0x00000800;       //  Enable PLL by clearing BYPASS
    
    
    SYSCTL_RCGCUART_R |= 0x04;  // provide clock to UART2
    SYSCTL_RCGCGPIO_R |= 0x8;  // Enable clock to PORTD
    
    SysTick_init();

    /* UART2 initialization */
    UART2_CTL_R = 0;         // disable UART2
    UART2_IBRD_R = 520;      // 80MHz/16=5MHz, 5MHz/520=>9600 baud rate
    UART2_FBRD_R = 53;       // fraction part, .8333*64 + 0.5
    UART2_CC_R = 0;          // use system clock
    UART2_LCRH_R = 0x60;     // 8-bit, no parity, 1-stop bit
    UART2_CTL_R = 0x301;     // enable UART2, TXE, RXE
    
    /* UART2 TX5 and RX5 use PC7 and PC6. Set them up. */
    GPIO_PORTD_LOCK_R = 0x4C4F434B;   //  Unlock the port register
    GPIO_PORTD_CR_R = 0xFF;           // Allow changes to PD7..PD0

    GPIO_PORTD_DEN_R |= 0xC0;          /* make PD7, PD6 as digital */
    GPIO_PORTD_AMSEL_R = 0;           /* turn off analog function */
    GPIO_PORTD_AFSEL_R = 0xC0;        /* use PD7, PD6 alternate function */
    GPIO_PORTD_PCTL_R = 0x11000000;   /* configure PD7, PD6 for UART2 */

	SysTick_delay();           /* wait for output line to stabilize */
    
    for(;;)
    {
		c = UART2Rx();       // blocking call - does not return until a char is input

			
			
		bool inputCharacterIsCapitalLetterInAscii = (c>= 65 && c<=90);
		bool inputCharacterIsLowercaseLetterInAscii = (c>= 97 && c<=122);
		//UART2Tx(c);
		if (inputCharacterIsCapitalLetterInAscii || inputCharacterIsLowercaseLetterInAscii )
		{
			if(inputCharacterIsCapitalLetterInAscii)
				{
					c= performShitfTransformationOnCapitalLetters(c);
				}
				
			if(inputCharacterIsLowercaseLetterInAscii)
				{
					c= performShitfTransformationOnLowercaseLetters(c);
				}
			
			numberOfKeystrokes++;
		
			
			displayDecimalNumberOnGpioBoardInBinary(numberOfKeystrokes);
			UART2Tx(c);
				
			}
       
       
    }   
}

/* UART2 Transmit */
void UART2Tx(char c)  
{
    while((UART2_FR_R & 0x20) != 0);     /* wait until Tx buffer not full */
    UART2_DR_R = c;                      /* before giving it another byte */
}

/* UART2 Transmit */
char UART2Rx(void)  
{
    char c;
    while((UART2_FR_R & 0x10) != 0);     /* wait until Rx buffer not empty */
    c = UART2_DR_R;                      /* read the char from the data register */
    return c;
}

void SysTick_init(void)
{
	NVIC_ST_CTRL_R = 0;							// 1. disable SysTick before configuring
	NVIC_ST_RELOAD_R = DELAY_VALUE;		        // 2. set to desired delay value
	NVIC_ST_CURRENT_R = 0;						// 3. clear CURRENT by writing any value
	NVIC_ST_CTRL_R &= ~0x00000004;		        // 4. set clock to POSC/4
	NVIC_ST_CTRL_R |= 0x00000001;			    // 5. enable SysTick timer
}

void SysTick_delay(void)
{
	NVIC_ST_CURRENT_R = 0;						// 1.	clear CURRENT by writing any value
	while((NVIC_ST_CTRL_R & 0x00010000)==0)		// 2. wait for count flag to be set
		{
		
		}
}


