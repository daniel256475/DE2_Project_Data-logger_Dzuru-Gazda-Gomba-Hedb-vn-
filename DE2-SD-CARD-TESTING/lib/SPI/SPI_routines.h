//**************************************************************
// ****** FUNCTIONS FOR SPI COMMUNICATION *******
//**************************************************************
//Controller        : ATmega32 (Clock: 8 Mhz-internal)
//Compiler          : AVR-GCC (winAVR with AVRStudio-4)
//Project Version   : DL_1.0
//Author            : CC Dharmani, Chennai (India)
//                    [www.dharmanitech.com](https://www.dharmanitech.com)
//Date              : 10 May 2011
//**************************************************************


#ifndef _SPI_ROUTINES_H_
#define _SPI_ROUTINES_H_


/**
 * @brief  Configure SPI for SD card communication (125 kHz clock rate).
 */
#define SPI_SD             SPCR = 0x52

/**
 * @brief  Configure SPI for high-speed communication with doubled clock rate.
 */
#define SPI_HIGH_SPEED     SPCR = 0x50; SPSR |= (1<<SPI2X)



/**
 * @brief  Initialize SPI module for SD card communication.
 * @return none
 */
void spi_init(void);

/**
 * @brief  Transmit one byte over SPI and receive response.
 * @param  data Byte to transmit.
 * @return Byte received during transmission.
 */
unsigned char SPI_transmit(unsigned char);

/**
 * @brief  Receive one byte over SPI.
 * @return Byte received from SPI.
 */
unsigned char SPI_receive(void);


#endif
