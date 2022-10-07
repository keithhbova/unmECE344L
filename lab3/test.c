#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#define COST_OF_ITEM 30

/************************************************************************/ 
/* ECE 344L - Microprocessors – Fall 2022   				   */ 
/*          								  */ 
/*         									   */ 
/* test.c -- implementation of vending machine logic on desktop for testing  */ 
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

/**********************************************************************************************************/


struct Input
{
	char storedCharacters[3];
};


/********************************************************************************************************/



struct VendingMachine
{
	short moneyInMachine;
};

struct User
{
	bool putsANickelInTheVendingMachine; 
	bool putsADimeInTheVendingMachine; 
	bool presesReset;
		
};



/***************************************************************************************************************/

void getInput(struct Input * input);
void initializeInput(struct Input * input);
void printMoneyInVendingMachine(struct VendingMachine * vendingMachine);
void assignInputToUser(struct User * user, struct Input * input);
bool checkIfInputIsValid(struct Input * input);
short indicateCurrentState(struct VendingMachine * vendingMachine);
void initializeUser(struct User * user);
void initializeVendingMachine(struct VendingMachine * vendingMachine);
void addANickelToTheVendingMachine(struct VendingMachine * vendingMachine);
void addADimeToTheVendingMachine(struct VendingMachine * vendingMachine);
bool checkIfEnoughMoneyHasBeenEnteredIntoVendingMachineToPurchaseItem(struct VendingMachine * vendingMachine);
void purchaseAnItemFromTheVendingMachine(struct VendingMachine * vendingMachine);
short dispenseChange(struct VendingMachine * vendingMachine);


/*******************************************************************************************************************/

void getInput(struct Input * input)
{
	scanf("%2s", input->storedCharacters);
}

void initializeInput(struct Input * input)
{
	strncpy(input->storedCharacters, "", sizeof(input->storedCharacters));
}

void printMoneyInVendingMachine(struct VendingMachine * vendingMachine)
{
	printf("there are %d\t cents in the machine\n", vendingMachine->moneyInMachine);
}

void assignInputToUser(struct User * user, struct Input * input)
{
	
	if(0 == strncmp(input->storedCharacters, "10", sizeof(input->storedCharacters)))
	{
		user->putsADimeInTheVendingMachine = true;
		return;
	}
	
	if(0 == strncmp(input->storedCharacters, "5", sizeof(input->storedCharacters)))
	{
		user->putsANickelInTheVendingMachine = true;
		return;
	}
}

bool checkIfInputIsValid(struct Input * input)
{
	if(0 == strncmp(input->storedCharacters, "10", sizeof(input->storedCharacters)))
	{
		return true;
	}
	if(0 == strncmp(input->storedCharacters, "5", sizeof(input->storedCharacters)))
	{
		return true;
	}
	return false;
}

short indicateCurrentState(struct VendingMachine * vendingMachine)
{
	short currentMoneyInMachine = vendingMachine->moneyInMachine;
	switch(currentMoneyInMachine)
	{
	case 0:
	{
		printf("no money\n");
		break;
	}
	case 5:
	{
		printf("5 cents\n");
		break;
	}
	case 10:
	{
		printf("10 cents\n");
		break;
	}
	case 15:
	{
		printf("15 cents\n");
		break;
	}
	case 20:
	{
		printf("20 cents\n");
		break;
	}
	case 25:
	{
		printf("25 cents\n");
		break;
	}
	case 30:
	{
		printf("30 cents\n");
		break;
	}
	}
	return currentMoneyInMachine;
}

/**************************************************************************/

void initializeUser(struct User * user)
{
	user->putsANickelInTheVendingMachine = false;
	user->putsADimeInTheVendingMachine = false;
	user->presesReset = false;
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

/**********************************************************************************************************************************/

int main()
{
	struct User user;
	struct Input input;
	struct VendingMachine vendingMachine;
	
	initializeVendingMachine(& vendingMachine);
	
	do
	{
		indicateCurrentState(& vendingMachine);
		
		initializeUser(& user);
		initializeInput(& input);
		
		getInput(& input);
		
		bool inputIsValid = checkIfInputIsValid(& input);
				
		if(inputIsValid)
		{
			assignInputToUser(& user, & input);
					
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
					
			bool userCanAffordAnItem = checkIfEnoughMoneyHasBeenEnteredIntoVendingMachineToPurchaseItem(& vendingMachine);
			if(userCanAffordAnItem)
			{
				purchaseAnItemFromTheVendingMachine(& vendingMachine);
				short change = dispenseChange(& vendingMachine);
				if(change)
				{
					printf("you got change\n");
				}
				
				break;
			}
					
		}
		
		
	}
	while(true);
	
	
	return 0;
}

