#include <stdio.h>
#include "NUC100Series.h"
#include "LCD.h"

#define HXT_STATUS 1<<0
#define PLL_STATUS 1<<2
#define TIMER1_COUNTS 200000 -1 // generating 0.5hz on timer 1
#define TIMER0_COUNTS 20 -1 // generating 50 Hz on timer 0

#define ON 1
#define OFF 0
#define WIN 2
#define LOSE 3
#define RESET 4

#define DEBOUNCE_DELAY 200000
#define CHANGE_MODE_KEY 9
#define LED_INDICATOR_PIN 15

void System_Config(void);
void SPI3_Config(void);
void timer1_config(void);
void timer0_config(void);

void LCD_start(void);
void LCD_command(unsigned char temp);
void LCD_data(unsigned char temp);
void LCD_clear(void);
void LCD_SetAddress(uint8_t PageAddr, uint8_t ColumnAddr);


uint8_t KeyPadScanning(void);
void KeyPadEnable(void);

//display system config
void seven_segment_config(void);
void displayNumberOfShot(int state);
void displayCol(void);
void displayRow(void);

// UART function and config
void UART0_Config(void);
void UART02_IRQHandler(void);

//menu function
void displayWelcomeMenu(void);

// map function
void loadMap(void);
void displayMap(void);
void resetMap(void);

//coordinate sytem config
void aimShip(void);

//shoot button config
void EINT1_Config(void);
void shootTheShip(void);

//declare variables

volatile int seg[] = {
  0b10000010,  //Number 0          // ---a----
	0b11101110,  //Number 1          // |      |
	0b00000111,  //Number 2          // f      b
	0b01000110,  //Number 3          // |      |
	0b01101010,  //Number 4          // ---g----
	0b01010010,  //Number 5          // |      |
	0b00010010,  //Number 6          // e      c
	0b11100110,  //Number 7          // |      |
	0b00000010,  //Number 8          // ---d----
	0b01000010,  //Number 9
	0b11111111,   //Blank LED
};

// display 7 segments variables
volatile int scanLED = 0;                                   // scan between leds
volatile int col_temp = 0;
// reading map from text file variable
/*volatile char map[150] = {0x30, 0x20, 0x30, 0x20, 0x30, 0x20, 0x30, 0x20, 0x30, 0x20, 0x30, 0x20, 0x30, 0x20, 0x30, 0x0A, 
												 0x30, 0x20, 0x31, 0x20, 0x31, 0x20, 0x30, 0x20, 0x30, 0x20, 0x30, 0x20, 0x30, 0x20, 0x30, 0x0A, 
												 0x30, 0x20, 0x30, 0x20, 0x30, 0x20, 0x30, 0x20, 0x30, 0x20, 0x30, 0x20, 0x31, 0x20, 0x30, 0x0A, 
												 0x30, 0x20, 0x30, 0x20, 0x30, 0x20, 0x30, 0x20, 0x30, 0x20, 0x30, 0x20, 0x31, 0x20, 0x30, 0x0A, 
												 0x30, 0x20, 0x30, 0x20, 0x31, 0x20, 0x31, 0x20, 0x30, 0x20, 0x30, 0x20, 0x30, 0x20, 0x30, 0x0A, 
												 0x30, 0x20, 0x30, 0x20, 0x30, 0x20, 0x30, 0x20, 0x30, 0x20, 0x30, 0x20, 0x30, 0x20, 0x30, 0x0A,
												 0x31, 0x20, 0x31, 0x20, 0x30, 0x20, 0x30, 0x20, 0x31, 0x20, 0x30, 0x20, 0x30, 0x20, 0x30, 0x0A,
												 0x30, 0x20, 0x30, 0x20, 0x30, 0x20, 0x30, 0x20, 0x31, 0x20, 0x30, 0x20, 0x30, 0x20, 0x30, 0x0A};*/

volatile char map[80];
volatile unsigned char loadedMap[9][9];
volatile int dataReceive = 0;
volatile char ReceivedByte;
volatile int isMapReady = 0;
// shooting variables
volatile int totalShot = 0;
volatile int numberOfDefeatShip = 0;
volatile int isShoot = 0;
volatile int isHit = 0;
volatile int countBlink = 0;
volatile int buzzer_count = 0;
volatile int isGameInit = 0;
volatile int numberOfHitShot = 0;
volatile int totalX = 0;

volatile int isGameOver = 0;
volatile int playAgainDisplay = 0;
volatile int timerCounter = 0; // Global counter for timing


// coordinating variables
volatile int isColSelected = 0;
volatile char mapElementState = '-';


// Structure
typedef struct Player // store the coordinate of player
{
	volatile int playerCol;
	volatile int playerRow;
} Player;

typedef struct Ship { // create ship
  volatile int firstPartCol;
  volatile int firstPartRow;
  volatile int secondPartCol;
  volatile int secondPartRow;
  volatile uint8_t firstPartShot;
  volatile uint8_t secondPartShot;
} Ship;
volatile Ship shipList[5];
volatile Player playerPos;


volatile int gameCheck;

int main(void)
{
	System_Config();

	SPI3_Config();
	LCD_start();
	LCD_clear();
	KeyPadEnable();
	UART0_Config();
	seven_segment_config();
	EINT1_Config();
	timer1_config();
	timer0_config();
	playerPos.playerCol = 1;
	playerPos.playerRow = 1;
	GPIO_SetMode(PC,BIT12,GPIO_MODE_OUTPUT);//hiting led
	GPIO_SetMode(PC,BIT15,GPIO_MODE_OUTPUT);//xy coordinating led
  GPIO_SetMode(PB,BIT11,GPIO_MODE_OUTPUT);//buzzer config
	
	displayWelcomeMenu();
	
	while (1) {
		switch(gameCheck)
{
    case OFF:
        // just to display
				
        break;

    case ON:
        if(isGameInit == 0)
        {
        
            LCD_clear();
            displayMap();
            isGameInit = 1;
        }
        displayNumberOfShot(1); // initially shot is 0 bcz total shot is 0
        aimShip();
        break;

    case RESET:
				resetMap();
        break;

    default:
        // Optionally handle any unexpected cases
        break;
}
	}
}

void System_Config(void) {
	SYS_UnlockReg(); // Unlock protected registers
	CLK->PWRCON |= (0x01 << 0);
	while (!(CLK->CLKSTATUS & (1 << 0)));

	//PLL configuration starts
	CLK->PLLCON &= ~(1 << 19); //0: PLL input is HXT
	CLK->PLLCON &= ~(1 << 16); //PLL in normal mode
	CLK->PLLCON &= (~(0x01FF << 0));
	CLK->PLLCON |= 48;
	CLK->PLLCON &= ~(1 << 18); //0: enable PLLOUT
	while (!(CLK->CLKSTATUS & (0x01ul << 2)));
	//PLL configuration ends

	//clock source selection
	CLK->CLKSEL0 &= (~(0x07 << 0));
	CLK->CLKSEL0 |= (0x02 << 0);
	//clock frequency division
	CLK->CLKDIV &= (~0x0F << 0);
	
	//UART0 Clock selection and configuration
	CLK->CLKSEL1 |= (0b11 << 24); // UART0 clock source is 22.1184 MHz
	CLK->CLKDIV &= ~(0xF << 8); // clock divider is 1
	CLK->APBCLK |= (1 << 16); // enable UART0 clock

	//enable clock of SPI3
	CLK->APBCLK |= 1 << 15;
	SYS_LockReg();  // Lock protected registers

}

void SPI3_Config(void) {
	SYS->GPD_MFP |= 1 << 11; //1: PD11 is configured for alternative func-tion
	SYS->GPD_MFP |= 1 << 9; //1: PD9 is configured for alternative function
	SYS->GPD_MFP |= 1 << 8; //1: PD8 is configured for alternative function

	SPI3->CNTRL &= ~(1 << 23); //0: disable variable clock feature
	SPI3->CNTRL &= ~(1 << 22); //0: disable two bits transfer mode
	SPI3->CNTRL &= ~(1 << 18); //0: select Master mode
	SPI3->CNTRL &= ~(1 << 17); //0: disable SPI interrupt
	SPI3->CNTRL |= 1 << 11; //1: SPI clock idle high
	SPI3->CNTRL &= ~(1 << 10); //0: MSB is sent first
	SPI3->CNTRL &= ~(3 << 8); //00: one transmit/receive word will be exe-cuted in one data transfer

	SPI3->CNTRL &= ~(31 << 3); //Transmit/Receive bit length
	SPI3->CNTRL |= 9 << 3;     //9: 9 bits transmitted/received per data transfer

	SPI3->CNTRL |= (1 << 2);  //1: Transmit at negative edge of SPI CLK
	SPI3->DIVIDER = 0; // SPI clock divider. SPI clock = HCLK / ((DIVID-ER+1)*2). HCLK = 50 MHz
}

void timer1_config(void)
{
		//TIMER1
  CLK->CLKSEL1 &= ~(0b111 <<12);  // clear the clock selection for timer 1
	CLK->CLKSEL1 |= (0b010 << 12); // use clock source of the CPU for the timer 1
	CLK->APBCLK |= (1<<3); // enable the clock for 1

	//reset timer-do before prescaler
	TIMER1->TCSR |= (1<<26);
	//set prescaler 11 for timer 1 
	TIMER1->TCSR &= ~(0xff << 0); // clear the TCSR register for timer 1
	TIMER1 ->TCSR |= 49 ; // set prescaler to 49 that reduce the clock frequency from 50Mhz to 1Mhz	
	
	//Mode periodic
	TIMER1->TCSR &= ~(0b11<<27);
	TIMER1->TCSR |= (0b01<<27);
	//BIT 24: USE TIMER AS COUNTER -> DISABLE
	TIMER1->TCSR &= ~(1<<24); //
	//value: update count to register
	TIMER1->TCSR |= (1<<16);
	TIMER1->TCSR |= (1 << 29);
	//set timer compare value
	// calculate the timer count value for timer 0
	// ((2s)/1/50Mhz) - 1 = 9999999999
	TIMER1->TCMPR = TIMER1_COUNTS;
	//ENABLE TIMER
	TIMER1->TCSR |= (1<<30);
	

	//Set Timer1 in NVIC Set-Enable Control Register (NVIC_ISER)
	NVIC->ISER[0] |= 1 << 9;
	//Priority for Timer 1
	NVIC->IP[2] &= (~(0b11<< 14));
	NVIC->IP[2] |= (0b01<< 14);
}

void timer0_config(void)
{
//Configure Timer 0 
	CLK->CLKSEL1 &= ~(0b111 <<8);  // clear the clock selection
	CLK->CLKSEL1 |= (0b010 << 8); // use clock source of the CPU for the timer
	CLK->APBCLK |= (1<<2); // enable the clock for timer
	
	//set prescaler 11 for timer 0 
	TIMER0->TCSR &= ~(0xff << 0); // clear the TCSR register
	TIMER0 ->TCSR |= 49 << 0; // set prescaler to 11 that reduce the clock frequency from 50Mhz to 1Mhz
	
	// bit 16 CRST to 1: reset prescaler counter, CEN and timer 0 counter
	TIMER0->TCSR |= (1<<0);
	
	// define Timer 0 operation mode
  TIMER0->TCSR &= ~(0b11 << 27);	//clear timer 0 MODE
	TIMER0->TCSR  |= 0b01<<27; //enable the periotic mode 01
	TIMER0->TCSR &= ~(1<<24);// Disable the counter mode of Timer 0
	
	//set the TDR to be updated continuously while timer counter 0 is counting
	TIMER0->TCSR |= (1<<16);
	
	// calculate the timer count value for timer 0
	// ((0.02s)/1/50Mhz) - 1 = 999
	
	// Set the reload value for timer 0
	TIMER0->TCMPR = TIMER0_COUNTS;
	
	// Configuring the polling Start
	// enable the TE (bit 29) 
	TIMER0->TCSR |= (1<<29);
	TIMER0->TCSR |= (1<<30);// start count   
//NVIC interrupt configuration for Timer 0 interrupt
	NVIC->ISER[0] |= 1<<8;
	NVIC->IP[2] &= ~(0b11 << 6);
	
}

void displayWelcomeMenu(void) {
	 printS_5x7(6,12, "Team B");
	printS_5x7(30,30, "Battle Ship Game");
}


void LCD_start(void)
{
	LCD_command(0xE2); // Set system reset
	LCD_command(0xA1); // Set Frame rate 100 fps
	LCD_command(0xEB); // Set LCD bias ratio E8~EB for 6~9 (min~max)
	LCD_command(0x81); // Set V BIAS potentiometer
	LCD_command(0xA0); // Set V BIAS potentiometer: A0 ()
	LCD_command(0xC0);
	LCD_command(0xAF); // Set Display Enable
}

void LCD_command(unsigned char temp)
{
	SPI3->SSR |= 1 << 0;
	SPI3->TX[0] = temp;
	SPI3->CNTRL |= 1 << 0;
	while (SPI3->CNTRL & (1 << 0));
	SPI3->SSR &= ~(1 << 0);
}


void LCD_data(unsigned char temp)
{
	SPI3->SSR |= 1 << 0;
	SPI3->TX[0] = 0x0100 + temp;
	SPI3->CNTRL |= 1 << 0;
	while (SPI3->CNTRL & (1 << 0));
	SPI3->SSR &= ~(1 << 0);
}


void LCD_clear(void)
{
	int16_t i;
	LCD_SetAddress(0x0, 0x0);
	for (i = 0; i < 132 * 8; i++)
	{
		LCD_data(0x00);
	}
	CLK_SysTickDelay(50000); // wait for all led is clear 
}



void LCD_SetAddress(uint8_t PageAddr, uint8_t ColumnAddr)
{
	LCD_command(0xB0 | PageAddr);
	LCD_command(0x10 | (ColumnAddr >> 4) & 0xF);
	LCD_command(0x00 | (ColumnAddr & 0xF));
}

void KeyPadEnable(void) {
	GPIO_SetMode(PA, BIT0, GPIO_MODE_QUASI);
	GPIO_SetMode(PA, BIT1, GPIO_MODE_QUASI);
	GPIO_SetMode(PA, BIT2, GPIO_MODE_QUASI);
	GPIO_SetMode(PA, BIT3, GPIO_MODE_QUASI);
	GPIO_SetMode(PA, BIT4, GPIO_MODE_QUASI);
	GPIO_SetMode(PA, BIT5, GPIO_MODE_QUASI);
}

uint8_t KeyPadScanning(void) {
	PA0 = 1; PA1 = 1; PA2 = 0; PA3 = 1; PA4 = 1; PA5 = 1;
	if (PA3 == 0) return 1;
	if (PA4 == 0) return 4;
	if (PA5 == 0) return 7;
	PA0 = 1; PA1 = 0; PA2 = 1; PA3 = 1; PA4 = 1; PA5 = 1;
	if (PA3 == 0) return 2;
	if (PA4 == 0) return 5;
	if (PA5 == 0) return 8;
	PA0 = 0; PA1 = 1; PA2 = 1; PA3 = 1; PA4 = 1; PA5 = 1;
	if (PA3 == 0) return 3;
	if (PA4 == 0) return 6;
	if (PA5 == 0) return 9;
	return 0;
}

void displayCol(void)
{
	if(gameCheck == OFF || gameCheck == RESET) {
		PE->DOUT = seg[0];
	}
	else {
		PE->DOUT = seg[playerPos.playerCol];
	}
}

void displayRow(void) {
	if(gameCheck == OFF || gameCheck == RESET) {
		PE->DOUT = seg[0];
	}
	else {
		PE->DOUT = seg[playerPos.playerRow];
	}
}
void aimShip(void)
{
	 int user_choice = 0;

    // Continuously scan for key input
    do {
        user_choice = KeyPadScanning();
        if (isShoot == 1) {
            isShoot = 0;
            return;
        }
    } while (user_choice == 0);

    // Debounce delay
    CLK_SysTickDelay(DEBOUNCE_DELAY);

    // Select row or column based on user input
    if (user_choice != CHANGE_MODE_KEY) {
        if (isColSelected) {
            playerPos.playerRow = user_choice; // Set player row
        } else {
            playerPos.playerCol = user_choice; // Set player column
        }
    } else {
        // Toggle between selecting row and column
        isColSelected = !isColSelected;
    }

    // Update LED indicator based on selection mode
    if (isColSelected) {
        PC->DOUT &= ~(1 << LED_INDICATOR_PIN); // LED on for vertical selection
    } else {
        PC->DOUT |= 1 << LED_INDICATOR_PIN;    // LED off for horizontal selection
    }

    // Refresh LCD display
    clear_LCD();
    displayMap();	
}


void seven_segment_config(void)
{
	// 7 seg config
  PC->PMD &= (~(0xFF<< 8));		
  PC->PMD |= (0b01010101 << 8);   //push pull for 4 leds
	
	//Set mode for PE0 to PE7
	PE->PMD &= (~(0xFFFF<< 0));		
	PE->PMD |= 0b0101010101010101<<0;   //set push pull for 7 segments

}

void EINT1_Config(void)
{
	//set external interrupt on button GPB15
	PB->PMD &= ~(0x03 << 30); // set The GPB15 as input
	PB->IMD &= ~(0x01 << 15); // set edge trigger interrupt
  PB->IEN |= (1<<15); // choose the falling Edge intterrupt
	
	//enable the debounce function
  // NVIC interrupt configuration for GPIO-B15 interrupt source
  NVIC->ISER[0] |= 1<<3;
  NVIC->IP[0] &= (~(3<<30));		
}


void shootTheShip(void) {
    numberOfDefeatShip = 0;
    isHit = 0; // Assume no hit initially
		

    // Check if any ship is hit
    for (int j = 0; j <= 4; j++) { // col -1 and row -1 to match from keypad to the index of ship
        if (playerPos.playerCol-1 == shipList[j].firstPartCol && playerPos.playerRow-1 == shipList[j].firstPartRow && shipList[j].firstPartShot == 1) { // check of the current part is already shot, doesnt count
            shipList[j].firstPartShot = 0; // Hit the first part of the ship
            isHit = 1;
						numberOfHitShot++;
            break;
        } else if (playerPos.playerCol-1 == shipList[j].secondPartCol && playerPos.playerRow-1 == shipList[j].secondPartRow && shipList[j].secondPartShot == 1) {
            shipList[j].secondPartShot = 0; // Hit the second part of the ship
            isHit = 1;
						numberOfHitShot++;
            break;
        }
    }
    // Count the number of defeated ships
    for (int j = 0; j <= 4; j++) {
        if (shipList[j].firstPartShot == 0 && shipList[j].secondPartShot == 0) {
            numberOfDefeatShip++;
        }
    }
    totalShot++;
    // Check game status
    if (numberOfDefeatShip >= 5 && totalShot <= 16) {
				isGameOver = 1;
				LCD_clear();
				printS_5x7(30,30, "YOU WIN");
				
    } else if (totalShot > 16) {
				isGameOver = 1;
				LCD_clear();
				printS_5x7(30,30, "YOU LOSE");
    }
		else {
			clear_LCD();
			displayMap();  // call this function again to load the ship list again and check any part of shit is shot and displayed
		}
 
}

void UART0_Config(void) {
	// UART0 pin configuration. PB.1 pin is for UART0 TX
	GPIO_SetMode(PB,BIT1,GPIO_MODE_OUTPUT); // set pb1 as output
	
	SYS->GPB_MFP |= (1 << 1); // GPB_MFP[1] = 1 -> PB.1 is UART0 TX pin
	
	SYS->GPB_MFP |= (1 << 0); // GPB_MFP[0] = 1 -> PB.0 is UART0 RX pin	
	PB->PMD &= ~(0b11 << 0);	// Set Pin Mode for GPB.0(RX - Input)

	// UART0 operation configuration
	UART0->LCR |= (0b11 << 0); // 8 data bit
	UART0->LCR &= ~(1 << 2); // one stop bit	
	UART0->LCR &= ~(1 << 3); // no parity bit
	UART0->FCR |= (1 << 1); // clear RX FIFO
	UART0->FCR |= (1 << 2); // clear TX FIFO
	UART0->FCR &= ~(0xF << 16); // FIFO Trigger Level is 1 byte]
	
	//Baud rate config: BRD/A = 1, DIV_X_EN=0
	//--> Mode 0, Baud rate = UART_CLK/[16*(A+2)] = 9600 bps
	UART0->BAUD &= ~(0b11 << 28); // mode 0	
	UART0->BAUD &= ~(0xFFFF << 0);
	UART0->BAUD |= 142;
	//config interupt for the uart
	NVIC->ISER[0] = 1<<12; // enable the control register enable the vector 28
	NVIC->IP[3] &= ~(1<<6); // set the interrupt priority of IRQ12
	UART0->IER |= (1<<0); // enable the uart interupt
}


void UART02_IRQHandler(void){
	
	char ReceivedByte;
	ReceivedByte = UART0->DATA;
	
	map[dataReceive] = ReceivedByte;
  dataReceive++;
	if(dataReceive > 77) {
				loadMap();
				LCD_clear();
				printS_5x7(8,30, "Map Loaded Successfully");
				isMapReady = 1;
				dataReceive=0;
	}
}
//load map function

void loadMap(void) {
    unsigned int rowIndex = 0, colIndex = 0;

    for (unsigned int i = 0; map[i] != '\0'; i++) {
        char currentChar = map[i];

        switch (currentChar) {
            case '0':
            case '1':
                loadedMap[rowIndex][colIndex] = currentChar;
                colIndex++;
                break;
            case '\n':
                rowIndex++;
                colIndex = 0;
                break;

            default:
                // Handle other characters or spaces
                break;
        }
    }
		
		// generate ship
		int shipIndex = 0;
    for (int row = 0; row < 8; row++) {
        for (int col = 0; col < 8; col++) {
            // Check if current cell is the first part of a ship
					//&& (shipList[shipIndex - 1].firstPartRow != row && shipList[shipIndex - 1].firstPartCol != col)
            if (loadedMap[row][col] == '1' ) { // check the current position is not overlapped
                // Initialize the first part of the ship
                shipList[shipIndex].firstPartRow = row;
                shipList[shipIndex].firstPartCol = col;
                shipList[shipIndex].firstPartShot = 1;

                // Check for the second part of the ship horizontally
                if (col + 1 < 8 && loadedMap[row][col + 1] == '1') {
                    shipList[shipIndex].secondPartRow = row;
                    shipList[shipIndex].secondPartCol = col + 1;
										shipList[shipIndex].secondPartShot = 1;
										loadedMap[row][col + 1] = '0'; // to avoild conflict 2 ship
                }
                // Check for the second part of the ship vertically
                else if (row + 1 < 8 && loadedMap[row + 1][col] == '1') {
                    shipList[shipIndex].secondPartRow = row + 1;
                    shipList[shipIndex].secondPartCol = col;
										shipList[shipIndex].secondPartShot = 1;
										loadedMap[row + 1][col] = '0';// to avoild conflict 2 ship
                }
                // If no second part found, continue to next cell
                else {
                    continue;
                }
                shipIndex++;
            }
        }
    }
}


void displayMap(void) {
    for (int row = 0; row < 8; row++) {
        for (int col = 0; col < 8 && loadedMap[row][col] != 0; col++) {
            char displayChar = '-';
            int col_pos = col * 17; // the starting position of column
						
							for (int k = 0; k <= 4; k++) { // check if any part of ship is shot the replaced with X
                if ((shipList[k].firstPartShot == 0 && shipList[k].firstPartCol == col && shipList[k].firstPartRow == row) ||
                    (shipList[k].secondPartShot == 0 && shipList[k].secondPartCol == col && shipList[k].secondPartRow == row)) {
                    displayChar = 'X';
                    break;
                }
            
						}

            printC_5x7(col_pos, row * 8, displayChar);
        }
    }
}

void displayNumberOfShot(int state) {
  int tens, units;

    if (totalShot >= 0 && totalShot < 100) { // Ensure totalShot is within the range 0-99
        tens = totalShot / 10;   // Get the tens digit
        units = totalShot % 10;  // Get the units digit
    }
		if(state == 1) {
			PE->DOUT = seg[tens]; // at the index of seg, the corresponding value for that seg
		}
		else if(state == 2) {
			PE->DOUT = seg[units];
		}
		//PE->DOUT = seg[tens]; // at the index of seg, the corresponding value for that seg
		//PE->DOUT = seg[units];
}


void TMR1_IRQHandler(void) { // to blink and buzzer since the frequency is low
	if(isHit)
	{
		if (countBlink < 6) {
            PC->DOUT ^= (1 << 12); // Toggle PC12
            countBlink++;
        } else {
            // Reset everything once blinking is done
            isHit = 0;
            countBlink = 0;
            PC->DOUT |= (1 << 12); // Ensure LED is turned off
        }
	}
	
	if(isGameOver && buzzer_count < 10)
	{
		PB->DOUT ^= 1<<11;
		buzzer_count++;
	}
	
	TIMER1->TISR |= (1 << 0); // clear timer1 interttupt flag
} 

void TMR0_IRQHandler(void) { // to display LED since the freq is low
		col_temp = isColSelected;
	//Turn on U11, while turn off other 7segments
		if(scanLED > 3) {
			scanLED = 0;
		}
		else {
			scanLED++;
		}
				PC->DOUT &= ~(1 << 7);
				PC->DOUT &= ~(1<<6);		//SC3
				PC->DOUT &= ~(1<<5);		//SC2
				PC->DOUT &= ~(1<<4);		//SC1
  switch(scanLED) {
    case 0: // toggle SC4
        PC->DOUT |= (1 << 7);
				PC->DOUT &= ~(1<<6);		//SC3
				PC->DOUT &= ~(1<<5);		//SC2
				PC->DOUT &= ~(1<<4);		//SC1
				if(isColSelected) {
					displayRow();
				}
				else {
					displayCol();
				}
        break;
    case 1: // toggle seg led 2
				PC->DOUT &= ~(1 << 7);
				PC->DOUT &= ~(1<<6);		//SC3
				PC->DOUT |= (1<<5);		//SC2
				PC->DOUT &= ~(1<<4);		//SC1
				displayNumberOfShot(scanLED);
        break;
    case 2: // toggle seg led 3
				PC->DOUT &= ~(1 << 7);
				PC->DOUT &= ~(1<<6);		//SC3
				PC->DOUT &= ~(1<<5);		//SC2
				PC->DOUT |= (1<<4);		//SC1
				displayNumberOfShot(scanLED);
        break;
    default:
        break;
}
	
	if(isGameOver == 1 && playAgainDisplay == 0) {
		if(timerCounter <  TIMER1_COUNTS) { // wait after 6s to print the play again message
			timerCounter++;
		}
		else {
			LCD_clear();
			playAgainDisplay = 1;
		}		
	}
	else if(isGameOver == 1 && playAgainDisplay == 1) {
				printS_5x7(30,30, "Play again?");
	}
	TIMER0->TISR |= (1 << 0); // clear timer0 interrupt flag
} 



void EINT1_IRQHandler(void) { 
		 CLK_SysTickDelay(150000); // Button debounce 

    // Using switch case for gameCheck handling
    switch(gameCheck) {
        case ON: // if the game is on, shoot the ship
            isShoot = 1;
            shootTheShip();
            break;

        case RESET:
            LCD_clear(); // Clear LCD
            loadMap(); // Load map again
            gameCheck = ON;
            break;

        default:
            break;
    }
		
		if(gameCheck == OFF && isMapReady) { // to prevent user press button when map is not transmitted
			LCD_clear();
			gameCheck = ON;
		}

    if(isGameOver && buzzer_count >= 10 && timerCounter >= TIMER1_COUNTS) { // press to reset the game
        // Reset all values
        LCD_clear(); // Clear LCD
        gameCheck = RESET;
    }
		

    PB->ISRC |= (1 << 15); // Clear external interrupt flag
}

void resetMap(void)
{
	isGameInit = 0;
  buzzer_count = 0;
  totalShot = 0;
  playerPos.playerCol = 1;
  playerPos.playerRow = 1;
  isColSelected = 0;
  dataReceive = 0;
  numberOfDefeatShip = 0;
	isGameOver = 0;
	timerCounter = 0;
	playAgainDisplay = 0;
	displayCol();
	displayRow();
	displayWelcomeMenu();
}

