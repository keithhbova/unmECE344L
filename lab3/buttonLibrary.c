
#include <stdint.h>
#include <stdbool.h>
#include <tm4c123gh6pm.h>
#define DELAY_VALUE 0xF423F				// 0x1E847F = 0.5 sec delay at 4MHz input clock
																	// OXF423F = 0.25 sec delay at 4MHz
#define COST_OF_ITEM 30

/************************************************************************/ 
/* ECE 344L - Microprocessors – Fall 2022   				   */ 
/*          								  */ 
/*         									   */ 
/* buttonLibrary.c -- implementation of vending machine logic on ti board  */ 
/*        									    */ 
/*     									       */ 
/************************************************************************/ 
/* Author(s):  keith bova     						 */ 
/*                                        				  */ 
/************************************************************************/ 
/*   Detailed File Description:     					   */ 
/*  given nickels and dimes, buy something 30 cents from a vending   	*/ 
/*  machine   								 */ 
/*           								 */ 
/************************************************************************/ 
/*  Revision History:  friday oct 7, 2022				   */ 
/*    (useful for your own purposes)  					  */ 
/*           									 */ 
/************************************************************************/ 


/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Button/GPIO Library%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/


void delayMs(int delayTime);
int input;


struct OutputLightsOnBoard
{
	bool turnOnRedLight;
	bool turnOnGreenLight;
	bool turnOnBlueLight;
	bool turnOnNothing;
};


struct ButtonInput
{
	bool leftSwitchPressed;
	bool	rightSwitchPressed;
	bool bothSwitchesPressed;
	bool nothingPressed;
};


void SysTick_delay(void){
	NVIC_ST_CURRENT_R = 0;										// 1.	clear CURRENT by writing any value
	while((NVIC_ST_CTRL_R & 0x00010000)==0)		// 2. wait for count flag to be set
		{
		}
}

void fetchButtonDataFromBoard(struct ButtonInput * myInput)
{
	input = GPIO_PORTF_DATA_R & 0x11;  // raw input bits - others masked
	
	switch (input){
			case 0:							// sw1 & sw2 pressed
				myInput->bothSwitchesPressed = true;
        break;
			case 1:                         // sw1 pressed
				myInput->leftSwitchPressed = true;
			  break;
      case 16:                        // sw2 pressed
				myInput->rightSwitchPressed = true;
				break;
      default:
				myInput->nothingPressed = true;
      }  
	SysTick_delay();
}

void executeLightDataOnBoard(struct OutputLightsOnBoard * lightsOnBoard)
{
	if(lightsOnBoard->turnOnBlueLight)
	{
		GPIO_PORTF_DATA_R = 4;			//light blue
	}
	if(lightsOnBoard->turnOnRedLight)
	{
		
		GPIO_PORTF_DATA_R = 2;      // light red
	}
	if(lightsOnBoard->turnOnGreenLight)
	{
		GPIO_PORTF_DATA_R = 8;      // light green
	}
	if(lightsOnBoard->turnOnNothing)
	{
		GPIO_PORTF_DATA_R = 0;			// lights out!
	}
}

void delayMs(int delayTime)
{
	for(int i = 0; i < delayTime; i++)
	{
		for(int j = 0; j < 3180; j++)
		{
			
		}
	}
}


void initializeClock()
{
	SYSCTL_RCGCGPIO_R |= 0x00000020; // (a) activate clock for port F
}

void resetLightsOnBoardToZeroFromLibrary()
{
	GPIO_PORTF_DIR_R |= 0x0E;         	/* make PORTF3, 2, 1 output for LEDs */
	GPIO_PORTF_DATA_R = 0;							// lights out!
}

void resetLightsOnBoardToZero(struct OutputLightsOnBoard * lightsOnBoard)
{
	lightsOnBoard->turnOnRedLight = false;
	lightsOnBoard->turnOnGreenLight = false;
	lightsOnBoard->turnOnBlueLight = false;
	lightsOnBoard->turnOnNothing = false;
}

void resetButtonsToZeroFromLibrary()
{
	GPIO_PORTF_LOCK_R = 0x4C4F434B;   /* unlock commit register */
  GPIO_PORTF_CR_R = 0x01;           /* make PORTF0 configurable */
  GPIO_PORTF_LOCK_R = 0;            /* lock commit register */

  /* configure PORTF for switch input and LED output */
  GPIO_PORTF_DIR_R &= ~0x11;        	/* make PORTF 4, 0 input for switch */
  
	GPIO_PORTF_AFSEL_R &= ~0x11;    	// disable alt funct on PF4, PF0
  GPIO_PORTF_DEN_R |= 0x1F;         	/* make PORTF4-0 digital pins */
  GPIO_PORTF_PUR_R |= 0x11;         	/* enable pull up for PORTF4, 0 */
}

void resetButtonsToZero(struct ButtonInput * myInput)
{
	myInput->leftSwitchPressed = false;
	myInput->rightSwitchPressed = false;
	myInput->bothSwitchesPressed = false;
	myInput->nothingPressed = false;
	resetButtonsToZeroFromLibrary();
}	

void SysTick_init(void){
	NVIC_ST_CTRL_R = 0;								// 1. disable SysTick before configuring
	NVIC_ST_RELOAD_R = DELAY_VALUE;		// 2. set to desired delay value
	NVIC_ST_CURRENT_R = 0;						// 3.	clear CURRENT by writing any value
	NVIC_ST_CTRL_R &= ~0x00000004;		// 4. set clock to POSC/4
	NVIC_ST_CTRL_R |= 0x00000001;			// 5. enable SysTick timer
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

void displayDecimalNumberOnBoardInBinary(int decimalToDisplayInBinary)
{
	delayMs(50);
	if(16 > decimalToDisplayInBinary)
	{
		GPIO_PORTE_DATA_R = decimalToDisplayInBinary & 0xF;          // lower 4 count bits - PE0..PE3
		GPIO_PORTD_DATA_R = (decimalToDisplayInBinary & 0xF0) >> 4;  // upper 4 count bits - PD0..PD3
	}
	
}

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Vending Machine Logic%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/



struct VendingMachine
{
	short moneyInMachine;
};

struct User
{
	bool putsANickelInTheVendingMachine; 
	bool putsADimeInTheVendingMachine; 
	bool presesReset;
	bool doesNothing;
		
};



/***************************************************************************************************************/


void printMoneyInVendingMachine(struct VendingMachine * vendingMachine);
void assignInputToUser(struct User * user, struct ButtonInput * input);
bool checkIfInputIsValid(struct ButtonInput * input);
short indicateCurrentState(struct VendingMachine * vendingMachine);
void resetUserToZero(struct User * user);
void initializeVendingMachine(struct VendingMachine * vendingMachine);
void addANickelToTheVendingMachine(struct VendingMachine * vendingMachine);
void addADimeToTheVendingMachine(struct VendingMachine * vendingMachine);
bool checkIfEnoughMoneyHasBeenEnteredIntoVendingMachineToPurchaseItem(struct VendingMachine * vendingMachine);
void purchaseAnItemFromTheVendingMachine(struct VendingMachine * vendingMachine);
short dispenseChange(struct VendingMachine * vendingMachine);


/*******************************************************************************************************************/





void printMoneyInVendingMachine(struct VendingMachine * vendingMachine)
{
	displayDecimalNumberOnBoardInBinary(vendingMachine->moneyInMachine);
}

void assignInputToUser(struct User * user, struct ButtonInput * input)
{
	
	if(input->leftSwitchPressed)
	{
		user->putsADimeInTheVendingMachine = true;
		return;
	}
	
	if(input->rightSwitchPressed)
	{
		user->putsANickelInTheVendingMachine = true;
		return;
	}
	if(input->bothSwitchesPressed)
	{
		user->presesReset = true;
		return;
	}
	if(input->nothingPressed)
	{
		user->doesNothing = true;
		return;
	}
}

bool checkIfInputIsValid(struct ButtonInput * input)
{
	return true;
}

short indicateCurrentState(struct VendingMachine * vendingMachine)
{
	short currentMoneyInMachine = vendingMachine->moneyInMachine;
	switch(currentMoneyInMachine)
	{
	case 0:
	{
		displayDecimalNumberOnBoardInBinary(0);
		break;
	}
	case 5:
	{
		displayDecimalNumberOnBoardInBinary(1);
		break;
	}
	case 10:
	{
		displayDecimalNumberOnBoardInBinary(2);
		break;
	}
	case 15:
	{
		displayDecimalNumberOnBoardInBinary(3);
		break;
	}
	case 20:
	{
		displayDecimalNumberOnBoardInBinary(4);
		break;
	}
	case 25:
	{
		displayDecimalNumberOnBoardInBinary(5);
		break;
	}
	case 30:
	{
		displayDecimalNumberOnBoardInBinary(6);
		break;
	}
	}
	return currentMoneyInMachine;
}

/**************************************************************************/

void resetUserToZero(struct User * user)
{
	user->putsANickelInTheVendingMachine = false;
	user->putsADimeInTheVendingMachine = false;
	user->presesReset = false;
	user->doesNothing = false;
}

void initializeVendingMachine(struct VendingMachine * vendingMachine)
{
	vendingMachine->moneyInMachine = 0;
}

void addANickelToTheVendingMachine(struct VendingMachine * vendingMachine)
{
	vendingMachine->moneyInMachine += 5;
}

void addADimeToTheVendingMachine(struct VendingMachine * vendingMachine)
{
	vendingMachine->moneyInMachine += 10;
}

bool checkIfEnoughMoneyHasBeenEnteredIntoVendingMachineToPurchaseItem(struct VendingMachine * vendingMachine)
{
	if(COST_OF_ITEM <= vendingMachine->moneyInMachine)
	{
		return true;
	}
	return false;
}

void purchaseAnItemFromTheVendingMachine(struct VendingMachine * vendingMachine)
{
	vendingMachine->moneyInMachine -= 30;
}

short dispenseChange(struct VendingMachine * vendingMachine)
{
	short change = vendingMachine->moneyInMachine;
	initializeVendingMachine(vendingMachine);
	return change;
}

void flashLightsToIndicateChange()
{
	displayDecimalNumberOnBoardInBinary(15);
	displayDecimalNumberOnBoardInBinary(0);
	displayDecimalNumberOnBoardInBinary(15);
	displayDecimalNumberOnBoardInBinary(0);
	displayDecimalNumberOnBoardInBinary(15);
	displayDecimalNumberOnBoardInBinary(0);
	displayDecimalNumberOnBoardInBinary(15);
	displayDecimalNumberOnBoardInBinary(0);
	displayDecimalNumberOnBoardInBinary(15);
	displayDecimalNumberOnBoardInBinary(0);
	displayDecimalNumberOnBoardInBinary(15);
	displayDecimalNumberOnBoardInBinary(0);
}


/**********************************************************************************************************************************/

int main()
{
	struct User user;
	struct ButtonInput myInput;
	struct OutputLightsOnBoard lightsOnBoard;
	struct VendingMachine vendingMachine;
	
	
	initializeClock();
	SysTick_init();
	initializeGPIOPorts();
	resetLightsOnBoardToZeroFromLibrary();
	
	initializeVendingMachine(& vendingMachine);
	
	do
	{
		indicateCurrentState(& vendingMachine);
		
		resetUserToZero(& user);
		resetButtonsToZero(& myInput);
		resetLightsOnBoardToZero(& lightsOnBoard);
		
		lightsOnBoard.turnOnRedLight = true;
		
		fetchButtonDataFromBoard(& myInput);
		
		bool inputIsValid = checkIfInputIsValid(& myInput);
				
		if(inputIsValid)
		{
			assignInputToUser(& user, & myInput);
					
			if(user.putsADimeInTheVendingMachine)
			{
				addADimeToTheVendingMachine(& vendingMachine);
			}
					
			if(user.putsANickelInTheVendingMachine)
			{
				addANickelToTheVendingMachine(& vendingMachine);
			}
					
			if(user.presesReset)
			{
				dispenseChange(& vendingMachine);
			}
			if(user.doesNothing)
			{
				
			}
			
			bool userCanAffordAnItem = checkIfEnoughMoneyHasBeenEnteredIntoVendingMachineToPurchaseItem(& vendingMachine);
			if(userCanAffordAnItem)
			{
				displayDecimalNumberOnBoardInBinary(8);
				lightsOnBoard.turnOnGreenLight = true;
				purchaseAnItemFromTheVendingMachine(& vendingMachine);
				short change = dispenseChange(& vendingMachine);
				if(change)
				{
					flashLightsToIndicateChange();
				}
			}	
		executeLightDataOnBoard(& lightsOnBoard);
		}
	}
	while(true);
	
	
	return 0;
}



/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%main%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/


/*
int main(void)
{
		struct ButtonInput myInput;
		struct OutputLightsOnBoard lightsOnBoard;
	
		initializeClock();
		SysTick_init( );
		initializeGPIOPorts();
		
    
    while(1)
    {
			resetButtonsToZero(& myInput);
			resetLightsOnBoardToZero(& lightsOnBoard);
			
			fetchButtonDataFromBoard(& myInput); 
			
			
			if(myInput.leftSwitchPressed)
			{
				lightsOnBoard.turnOnRedLight = true;
				displayDecimalNumberOnBoardInBinary(1);
				
			}
			
			if(myInput.rightSwitchPressed)
			{
				lightsOnBoard.turnOnGreenLight = true;
				displayDecimalNumberOnBoardInBinary(2);
			}
			if(myInput.bothSwitchesPressed)
			{
				lightsOnBoard.turnOnBlueLight = true;
				displayDecimalNumberOnBoardInBinary(3);
			}
			if(myInput.nothingPressed)
			{
				displayDecimalNumberOnBoardInBinary(0);
			}
			
			
			
			executeLightDataOnBoard(& lightsOnBoard);
			
	}
	
}
*/
