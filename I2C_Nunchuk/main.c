#include <stdio.h>
#include <string.h>
#include "stm32f05xxx.h"
#include "stm32f05xxx_i2c.h"
#include "nrf24.h"

#define F_CPU 48000000L
#define NUNCHUK_ADDRESS 0x52

// These two functions are in serial.c
int egets(char *s, int Max);
int serial_data_available(void);

void wait_1ms(void)
{
	// For SysTick info check the STM32F0xxx Cortex-M0 programming manual page 85.
	STK_RVR = (F_CPU/1000L) - 1;  // set reload register, counter rolls over from zero, hence -1
	STK_CVR = 0; // load the SysTick counter
	STK_CSR = 0x05; // Bit 0: ENABLE, BIT 1: TICKINT, BIT 2:CLKSOURCE
	while((STK_CSR & BIT16)==0); // Bit 16 is the COUNTFLAG.  True when counter rolls over from zero.
	STK_CSR = 0x00; // Disable Systick counter
}

void delayms(int len)
{
	while(len--) wait_1ms();
}

int I2C_byte_write(unsigned char saddr, unsigned char maddr, unsigned char data)
{
	I2C1_CR1 = I2C_CR1_PE;
	I2C1_CR2 = I2C_CR2_AUTOEND | (2 << 16) | (saddr << 1);
	I2C1_CR2 |= I2C_CR2_START;
	while ((I2C1_ISR & I2C_ISR_TXE) != I2C_ISR_TXE) {} // Wait for address to go

	I2C1_TXDR = maddr; // send data
	while ((I2C1_ISR & I2C_ISR_TXE) != I2C_ISR_TXE) {} // Check Tx empty
	
	I2C1_TXDR = data; // send data
	while ((I2C1_ISR & I2C_ISR_TXE) != I2C_ISR_TXE) {} // Check Tx empty

	return 0;
}

int I2C_burst_write(unsigned char saddr, unsigned char maddr, int byteCount, unsigned char* data)
{
	I2C1_CR1 = I2C_CR1_PE;
	I2C1_CR2 = I2C_CR2_AUTOEND | ((byteCount+1) << 16) | (saddr << 1);
	I2C1_CR2 |= I2C_CR2_START;
	while ((I2C1_ISR & I2C_ISR_TXE) != I2C_ISR_TXE) {} // Wait for address to go

	I2C1_TXDR = maddr; // send data
	while ((I2C1_ISR & I2C_ISR_TXE) != I2C_ISR_TXE) {} // Check Tx empty

    for (; byteCount > 0; byteCount--)
    {
		I2C1_TXDR = *data++; // send data
		while ((I2C1_ISR & I2C_ISR_TXE) != I2C_ISR_TXE) {} // Check Tx empty
	}

	return 0;
}

int I2C_burstRead(unsigned char saddr, char maddr, int byteCount, unsigned char* data)
{
	// First we send the address we want to read from:
	I2C1_CR1 = I2C_CR1_PE;
	I2C1_CR2 = I2C_CR2_AUTOEND | (1 << 16) | (saddr << 1);
	I2C1_CR2 |= I2C_CR2_START; // Go
	while ((I2C1_ISR & I2C_ISR_TXE) != I2C_ISR_TXE) {} // Wait for address to go

	I2C1_TXDR = maddr; // send data
	while ((I2C1_ISR & I2C_ISR_TXE) != I2C_ISR_TXE) {} // Check Tx empty
	
	// Second: we gatter the data sent by the slave device
	I2C1_CR1 = I2C_CR1_PE | I2C_CR1_RXIE;
	I2C1_CR2 = I2C_CR2_AUTOEND | (byteCount<<16) | I2C_CR2_RD_WRN | (NUNCHUK_ADDRESS << 1);
	I2C1_CR2 |= I2C_CR2_START; // Go
    
    for (; byteCount > 0; byteCount--)
    {
		while ((I2C1_ISR & I2C_ISR_RXNE) != I2C_ISR_RXNE) {} // Wait for data to arrive
		*data++=I2C1_RXDR; // Reading 'receive' register clears RXNE flag
	}

	return 0;
}

void nunchuck_init(int print_extension_type)
{
	unsigned char buf[6];
	
	I2C_byte_write(0x52, 0xF0, 0x55);
	I2C_byte_write(0x52, 0xFB, 0x00);
		 
	// Read the extension type from the register block.
	// For the original Nunchuk it should be: 00 00 a4 20 00 00.
	I2C_burstRead(0x52, 0xFA, 6, buf);
	if(print_extension_type)
	{
		printf("Extension type: %02x  %02x  %02x  %02x  %02x  %02x\r\n", 
			buf[0],  buf[1], buf[2], buf[3], buf[4], buf[5]);
	}

	// Send the crypto key (zeros), in 3 blocks of 6, 6 & 4.
	buf[0]=0; buf[1]=0; buf[2]=0; buf[3]=0; buf[4]=0; buf[5]=0;
	
	I2C_byte_write(0x52, 0xF0, 0xAA);
	I2C_burst_write(0x52, 0x40, 6, buf);
	I2C_burst_write(0x52, 0x40, 6, buf);
	I2C_burst_write(0x52, 0x40, 4, buf);
}

void nunchuck_getdata(unsigned char * s)
{
	unsigned char i;

	// Start measurement
	I2C_burstRead(0x52, 0x00, 6, s);

	// Decrypt received data
	for(i=0; i<6; i++)
	{
		s[i]=(s[i]^0x17)+0x17;
	}
}

void I2C_init (void)
{
	RCC_AHBENR |= BIT18; // peripheral clock enable for port B (I2C pins are in port B)
	RCC_APB1ENR  |= BIT21; // peripheral clock enable for I2C1 (page 123 of RM0091 Reference manual)
	
	//Configure PB6 for I2C1_SCL, pin 29 in LQFP32 package (page 36 of Datasheet)
	GPIOB_MODER    |= BIT13; // AF-Mode (page 157 of RM0091 Reference manual)
	GPIOB_AFRL     |= BIT24; // AF1 selected (page 161 of RM0091 Reference manual)
	
	//Configure PB7 for I2C1_SDA, pin 30 in LQFP32 package
	GPIOB_MODER    |= BIT15; // AF-Mode (page 157 of RM0091 Reference manual)
	GPIOB_AFRL     |= BIT28; // AF1 selected (page 161 of RM0091 Reference manual)
	
	// This, somehow configures the I2C clock
	I2C1_TIMINGR = (uint32_t)0x00B01A4B;
}


// https://community.st.com/s/question/0D50X00009XkXwy/stm32f0-spi-read-and-write
static void config_SPI(void)
{
	RCC_AHBENR |= BIT17;  // Enable GPIOA clock
	RCC_APB2ENR |= BIT12; // peripheral clock enable for SPI1 (page 122 of RM0091 Reference manual)
	
	// Configure PA3 for CE, pin 9 in LQFP32 package
	GPIOA_MODER |= BIT6; // Make pin PA3 output
	GPIOA_ODR |= BIT3; // CE=1
	
	// Configure PA4 for CSn, pin 10 in LQFP32 package
	GPIOA_MODER |= BIT8; // Make pin PA4 output
	GPIOA_ODR |= BIT4; // CSn=1
	
	//Configure PA5 for SPI1_SCK, pin 11 in LQFP32 package
	GPIOA_MODER |= BIT11; // AF-Mode (page 157 of RM0091 Reference manual)
	GPIOA_AFRL  |= 0; // AF0 selected (page 161 of RM0091 Reference manual)
	
	//Configure PA6 for SPI1_MISO, pin 12 in LQFP32 package
	GPIOA_MODER |= BIT13; // AF-Mode (page 157 of RM0091 Reference manual)
	GPIOA_AFRL  |= 0; // AF0 selected (page 161 of RM0091 Reference manual)
	
	//Configure PA7 for SPI1_MOSI, pin 13 in LQFP32 package
	GPIOA_MODER |= BIT15; // AF-Mode (page 157 of RM0091 Reference manual)
	GPIOA_AFRL  |= 0; // AF0 selected (page 161 of RM0091 Reference manual)
	
	SPI1_CR1 = 0x00000000; // Reset SPI1 CR1 Registry.  Page 801 of RM0091 Reference manual
	SPI1_CR1 = (( 0ul << 0) | // CPHA=0 (the nRF24L01 samples data in the falling edge of the clock which works oke for mode (0,0) in the STM32!)
				( 0ul << 1) | // CPOL=0
				( 1ul << 2) | // MSTR=1
				( 7ul << 3) | // BR (fPCLK/256) ~= 187 kbit/sec ]
				( 0ul << 7) | // MSBFIRST
				( 1ul << 8) | // SSI must be 1 when SSM=1 or a frame error occurs
				( 1ul << 9)); // SSM
	SPI1_CR2 = ( BIT10 | BIT9 | BIT8 ); // 8-bits at a time (page 803 of RM0091 Reference manual)
	SPI1_CR1 |= BIT6; // Enable SPI1
}

uint8_t spi_transfer(uint8_t tx)
{
	uint8_t data=0;
	
	while ((SPI1_SR & BIT1) == 0); // SPI status register (SPIx_SR) is in page 806
	*((uint8_t *)&SPI1_DR) = tx; // "SPI1_DR = wr;" sends 16 bits instead of 8, that is why we are type-casting
	while (SPI1_SR & BIT7); // Check Busy flag (Page 806)
	//while ((SPI1_SR & BIT0) == 0); // 0: Rx buffer empty (hangs here)
	data = *((uint8_t *)&SPI1_DR); // "data = SPI1_DR;" waits for 16-bits instead of 8, that is why we are type-casting
	
	return data;
}

uint8_t temp;
uint8_t data_array[32];
uint8_t tx_address[] = "TXADD";
uint8_t rx_address[] = "RXADD";
uint8_t mode = 0;

void main(void)
{
	unsigned char rbuf[6];
 	int joy_x, joy_y, off_x, off_y, acc_x, acc_y, acc_z;
 	char but1, but2;

	printf("\x1b[2J\x1b[1;1H"); // Clear screen using ANSI escape sequence.
	printf ("STM32F051 I2C WII Nunchuck test program\r\n"
	        "File: %s\r\n"
	        "Compiled: %s, %s\r\n\r\n",
	        __FILE__, __DATE__, __TIME__);
	
	//NUNCHUCK STUFF
	I2C_init();
	nunchuck_init(1);
	delayms(100);
	nunchuck_getdata(rbuf);
	off_x=(int)rbuf[0]-128;
	off_y=(int)rbuf[1]-128;
	printf("Offset_X:%4d Offset_Y:%4d\r\n", off_x, off_y);
	
	//SPI STUFF
	config_SPI();
	// Use PA8 (pin 18) for pushbutton input
	GPIOA_MODER &= ~(BIT16 | BIT17); // Make pin PA8 input
	// Activate pull up for pin PA8:
	GPIOA_PUPDR |= BIT16; 
	GPIOA_PUPDR &= ~(BIT17);
	// Use PA2 (pin 8) for transmitter/receiver selection input
	GPIOA_MODER &= ~(BIT4 | BIT5); // Make pin PA2 input
	// Activate pull up for pin PA2:
	GPIOA_PUPDR |= BIT4; 
	GPIOA_PUPDR &= ~(BIT5);
	config_SPI();
    nrf24_init(); // init hardware pins
    nrf24_config(120,32); // Configure channel and payload size

    /* Set the device addresses */
    if(GPIOA_IDR&BIT2)
    {
    	printf("Set as transmitter\r\n");
	    nrf24_tx_address(tx_address);
	    nrf24_rx_address(rx_address);
    }
    else
    {
    	printf("Set as receiver\r\n");
	    nrf24_tx_address(rx_address);
	    nrf24_rx_address(tx_address);
    }

	while(1)
	{	
		//NUNCHUCK WHILE LOOOP
		nunchuck_getdata(rbuf);
		joy_x=(int)rbuf[0]-128-off_x;
		joy_y=(int)rbuf[1]-128-off_y;
		acc_x=rbuf[2]*4; 
		acc_y=rbuf[3]*4;
		acc_z=rbuf[4]*4;
		but1=(rbuf[5] & 0x01)?1:0;
		but2=(rbuf[5] & 0x02)?1:0;
		if (rbuf[5] & 0x04) acc_x+=2;
		if (rbuf[5] & 0x08) acc_x+=1;
		if (rbuf[5] & 0x10) acc_y+=2;
		if (rbuf[5] & 0x20) acc_y+=1;
		if (rbuf[5] & 0x40) acc_z+=2;
		if (rbuf[5] & 0x80) acc_z+=1;
		
		if (joy_y > 50)			//forwards
			mode = 1;
		else if(joy_y < -50)	//backwards
			mode = 2;
		else if(joy_x > 50)		//right
			mode = 3;
		else if(joy_x < -50)	//left
			mode = 4;
		else if(but1 == 0){		//z is pressed
			delayms(50);
			if(but1==0)
				mode=5;
		}
		else if(but2 == 0){		//c is pressed
			delayms(50);
			if(but2==0)
				mode=6;
		}
		else mode=0;			//nothing pressed, so do nothing.
		
		//SPI WHILE LOOP
		if(nrf24_dataReady())
        {
            nrf24_getData(data_array);
        	printf("IN: %s\r\n", data_array);
        }
        
        if(serial_data_available()) // Did something arrived from the serial port?
        {
        	egets(data_array, sizeof(data_array));
		    printf("\r\n");    
	        nrf24_send(data_array);        
		    while(nrf24_isSending());
		    temp = nrf24_lastMessageStatus();
			if(temp == NRF24_MESSAGE_LOST)
		    {                    
		        printf("> Message lost\r\n");    
		    }
			nrf24_powerDown();
    		nrf24_powerUpRx();
		}
		
		if((GPIOA_IDR&BIT8)==0)
		{
			while((GPIOA_IDR&BIT8)==0);
			strcpy(data_array, "Button test");
	        nrf24_send(data_array);
		    while(nrf24_isSending());
		    temp = nrf24_lastMessageStatus();
			if(temp == NRF24_MESSAGE_LOST)
		    {                    
		        printf("> Message lost\r\n");    
		    }
			nrf24_powerDown();
    		nrf24_powerUpRx();
		}
		
		
		
		printf("Buttons(Z:%c, C:%c) Joystick(%4d, %4d) Accelerometer(%3d, %3d, %3d)\x1b[0J\r",
			   but1?'1':'0', but2?'1':'0', joy_x, joy_y, acc_x, acc_y, acc_z);
		fflush(stdout);
		delayms(100);
	}
	
}
