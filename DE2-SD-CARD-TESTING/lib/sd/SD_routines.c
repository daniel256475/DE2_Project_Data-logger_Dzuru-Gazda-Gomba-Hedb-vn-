//**************************************************************
// ****** FUNCTIONS FOR SD RAW DATA TRANSFER *******
//**************************************************************
//Controller		: ATmega32 (Clock: 8 Mhz-internal)
//Compiler			: AVR-GCC (winAVR with AVRStudio-4)
//Project Version	: DL_1.0
//Author			: CC Dharmani, Chennai (India)
//			  		  www.dharmanitech.com
//Date				: 10 May 2011
//**************************************************************

//Link to the Post: http://www.dharmanitech.com/2009/01/sd-card-interfacing-with-atmega8-fat32.html

#include <avr/io.h>
#include <avr/pgmspace.h>
#include "SPI_routines.h"
#include "sd_routines.h"
#include <uart.h>
#include <util/delay.h>

// Global variables
volatile unsigned char SDHC_flag = 0;
volatile unsigned char cardType = 0;
volatile unsigned char buffer[512];

/**
 * @brief  Initialize SD/SDHC card in SPI mode.
 * 
 * Performs card initialization sequence including idle state reset, version detection,
 * and configuration. Supports both SD and SDHC cards.
 * 
 * @return 0 on success, non-zero error code on failure.
 */
unsigned char SD_init(void)
{
unsigned char i, response, SD_version;
unsigned int retry=0 ;

 for(i=0;i<10;i++)
    SPI_transmit(0xff);

SD_CS_ASSERT;
do
{
  
   response = SD_sendCommand(GO_IDLE_STATE, 0); //send 'reset & go idle' command
   retry++;
   if(retry>0xFF) 
   {
      uart_puts("SD_CS_DEASSERT\r\n");
      SD_CS_DEASSERT;
      return 1;
   }
   
} while(response != 0x01);

SD_CS_DEASSERT;
SPI_transmit (0xff);
SPI_transmit (0xff);

retry = 0;

SD_version = 2; //default set to SD compliance with ver2.x; 
                //this may change after checking the next command
do
{
response = SD_sendCommand(SEND_IF_COND,0x000001AA); //Check power supply status, mendatory for SDHC card
retry++;
if(retry>0xfe) 
   {
      SD_version = 1;
      cardType = 1;
      break;
   } //time out

}while(response != 0x01);

retry = 0;

do
{
response = SD_sendCommand(APP_CMD,0); //CMD55, must be sent before sending any ACMD command
response = SD_sendCommand(SD_SEND_OP_COND,0x40000000); //ACMD41

retry++;
if(retry>0xfe) 
   {
      uart_puts("SD_SEND_OP_COND timeout\r\n");
      return 1;
   } 

}while(response != 0x00);


retry = 0;
SDHC_flag = 0;

if (SD_version == 2)
{ 
   do
   {
      response = SD_sendCommand(READ_OCR,0);
      retry++;
      if(retry>0xfe) 
      {
         cardType = 0;
         break;
      }

   }while(response != 0x00);

   if(SDHC_flag == 1) cardType = 2;
   else cardType = 3;
}

//SD_sendCommand(CRC_ON_OFF, OFF); //disable CRC; deafault - CRC disabled in SPI mode
//SD_sendCommand(SET_BLOCK_LEN, 512); //set block size to 512; default size is 512

SPI_HIGH_SPEED;

return 0; //successful return
}

/**
 * @brief  Send command to SD card with 32-bit argument.
 * 
 * Handles address conversion for SD (byte addressing) vs SDHC (block addressing).
 * Processes OCR register for SDHC detection.
 * 
 * @param  cmd  8-bit SD command code.
 * @param  arg  32-bit command argument.
 * @return Response byte from SD card.
 */
unsigned char SD_sendCommand(unsigned char cmd, unsigned long arg)
{
   
unsigned char response, retry=0, status;
// uart_puts_P("In SD_sendCommand\r\n");
//SD card accepts byte address while SDHC accepts block address in multiples of 512
//so, if it's SD card we need to convert block address into corresponding byte address by 
//multipying it with 512. which is equivalent to shifting it left 9 times
//following 'if' loop does that

if(SDHC_flag == 0)	
{
   uart_puts_P("SDHC_flag == 0\r\n");	
if(cmd == READ_SINGLE_BLOCK     ||
   cmd == READ_MULTIPLE_BLOCKS  ||
   cmd == WRITE_SINGLE_BLOCK    ||
   cmd == WRITE_MULTIPLE_BLOCKS ||
   cmd == ERASE_BLOCK_START_ADDR|| 
   cmd == ERASE_BLOCK_END_ADDR ) 
   {
      arg = arg << 9;
   }	   
}
SD_CS_ASSERT;
uart_puts_P("After SD_CS_ASSERT\r\n");
_delay_ms(1); //small delay before sending command
SPI_transmit(cmd | 0x40); //send command, first two bits always '01'
SPI_transmit(arg>>24);
SPI_transmit(arg>>16);
SPI_transmit(arg>>8);
SPI_transmit(arg);
uart_puts_P("CMDs\r\n");
if(cmd == SEND_IF_COND)	 //it is compulsory to send correct CRC for CMD8 (CRC=0x87) & CMD0 (CRC=0x95)
  SPI_transmit(0x87);    //for remaining commands, CRC is ignored in SPI mode
else 
  SPI_transmit(0x95); 

while((response = SPI_receive()) == 0xff) //wait response
   if(retry++ > 0xfe) break; //time out error

if(response == 0x00 && cmd == 58)  //checking response of CMD58
{
  status = SPI_receive() & 0x40;     //first byte of the OCR register (bit 31:24)
  if(status == 0x40) SDHC_flag = 1;  //we need it to verify SDHC card
  else SDHC_flag = 0;

  SPI_receive(); //remaining 3 bytes of the OCR register are ignored here
  SPI_receive(); //one can use these bytes to check power supply limits of SD
  SPI_receive(); 
}

SPI_receive(); //extra 8 CLK
SD_CS_DEASSERT;

return response; //return state
}

/**
 * @brief  Erase specified number of blocks from SD card.
 * @param  startBlock   Starting block address.
 * @param  totalBlocks  Number of blocks to erase.
 * @return 0 on success, error code on failure.
 */
unsigned char SD_erase (unsigned long startBlock, unsigned long totalBlocks)
{
unsigned char response;

response = SD_sendCommand(ERASE_BLOCK_START_ADDR, startBlock); //send starting block address
if(response != 0x00) //check for SD status: 0x00 - OK (No flags set)
  return response;

response = SD_sendCommand(ERASE_BLOCK_END_ADDR,(startBlock + totalBlocks - 1)); //send end block address
if(response != 0x00)
  return response;

response = SD_sendCommand(ERASE_SELECTED_BLOCKS, 0); //erase all selected blocks
if(response != 0x00)
  return response;

return 0; //normal return
}

/**
 * @brief  Read single 512-byte block from SD card into global buffer.
 * @param  startBlock Block address to read.
 * @return 0 on success, error code on failure.
 */
unsigned char SD_readSingleBlock(unsigned long startBlock)
{
   uart_puts_P("In SD_readSingleBlock\r\n");
   // uart_pu
unsigned char response;
unsigned int i, retry=0;
uart_puts_P("re1\r\n");
 response = SD_sendCommand(READ_SINGLE_BLOCK, startBlock); //read a Block command
 uart_puts_P("re2\r\n");
 if(response != 0x00) return response; //check for SD status: 0x00 - OK (No flags set)

SD_CS_ASSERT;
_delay_ms(1); //small delay before reading data
retry = 0;
while(SPI_receive() != 0xfe) //wait for start block token 0xfe (0x11111110)
  if(retry++ > 0xfffe){SD_CS_DEASSERT; return 1;} //return if time-out

for(i=0; i<512; i++) //read 512 bytes
  buffer[i] = SPI_receive();

SPI_receive(); //receive incoming CRC (16-bit), CRC is ignored here
SPI_receive();

SPI_receive(); //extra 8 clock pulses
SD_CS_DEASSERT;

return 0;
}

/**
 * @brief  Write single 512-byte block from global buffer to SD card.
 * @param  startBlock Block address to write.
 * @return 0 on success, error code on failure.
 */
unsigned char SD_writeSingleBlock(unsigned long startBlock)
{
unsigned char response;
unsigned int i, retry=0;

 response = SD_sendCommand(WRITE_SINGLE_BLOCK, startBlock); //write a Block command
  
 if(response != 0x00) return response; //check for SD status: 0x00 - OK (No flags set)

SD_CS_ASSERT;

SPI_transmit(0xfe);     //Send start block token 0xfe (0x11111110)

for(i=0; i<512; i++)    //send 512 bytes data
  SPI_transmit(buffer[i]);

SPI_transmit(0xff);     //transmit dummy CRC (16-bit), CRC is ignored here
SPI_transmit(0xff);

response = SPI_receive();

if( (response & 0x1f) != 0x05) //response= 0xXXX0AAA1 ; AAA='010' - data accepted
{                              //AAA='101'-data rejected due to CRC error
  SD_CS_DEASSERT;              //AAA='110'-data rejected due to write error
  return response;
}

while(!SPI_receive()) //wait for SD card to complete writing and get idle
if(retry++ > 0xfffe){SD_CS_DEASSERT; return 1;}

SD_CS_DEASSERT;
SPI_transmit(0xff);   //just spend 8 clock cycle delay before reasserting the CS line
SD_CS_ASSERT;         //re-asserting the CS line to verify if card is still busy

while(!SPI_receive()) //wait for SD card to complete writing and get idle
   if(retry++ > 0xfffe){SD_CS_DEASSERT; return 1;}
SD_CS_DEASSERT;

return 0;
}


#ifndef FAT_TESTING_ONLY

/**
 * @brief  Read multiple 512-byte blocks from SD card and output via UART.
 * @param  startBlock  Starting block address.
 * @param  totalBlocks Number of blocks to read.
 * @return 0 on success, error code on failure.
 */
unsigned char SD_readMultipleBlock (unsigned long startBlock, unsigned long totalBlocks)
{
unsigned char response;
unsigned int i, retry=0;

retry = 0;

response = SD_sendCommand(READ_MULTIPLE_BLOCKS, startBlock); //write a Block command
  
if(response != 0x00) return response; //check for SD status: 0x00 - OK (No flags set)

SD_CS_ASSERT;

while( totalBlocks )
{
  retry = 0;
  while(SPI_receive() != 0xfe) //wait for start block token 0xfe (0x11111110)
  if(retry++ > 0xfffe){SD_CS_DEASSERT; return 1;} //return if time-out

  for(i=0; i<512; i++) //read 512 bytes
     buffer[i] = SPI_receive();

  SPI_receive(); //receive incoming CRC (16-bit), CRC is ignored here
  SPI_receive();

  SPI_receive(); //extra 8 cycles
  uart_puts("\r\n");
  uart_puts(" --------- ");
  uart_puts("\r\n");

  for(i=0; i<512; i++) //send the block to UART
  {
     uart_putc(buffer[i]);
  }

  uart_puts("\r\n");
  uart_puts(" --------- ");
  uart_puts("\r\n");
  totalBlocks--;
}

SD_sendCommand(STOP_TRANSMISSION, 0); //command to stop transmission
SD_CS_DEASSERT;
SPI_receive(); //extra 8 clock pulses

return 0;
}

/**
 * @brief  Write multiple 512-byte blocks to SD card from UART input.
 * 
 * Receives data from UART (terminated with '~') and writes blocks to SD card.
 * 
 * @param  startBlock  Starting block address.
 * @param  totalBlocks Number of blocks to write.
 * @return 0 on success, error code on failure.
 */
unsigned char SD_writeMultipleBlock(unsigned long startBlock, unsigned long totalBlocks)
{
unsigned char response, data;
unsigned int i, retry=0;
unsigned long blockCounter=0;

response = SD_sendCommand(WRITE_MULTIPLE_BLOCKS, startBlock); //write a Block command

if(response != 0x00) return response; //check for SD status: 0x00 - OK (No flags set)

SD_CS_ASSERT;

uart_puts("\r\n");
uart_puts(" Enter text (End with ~): ");
uart_puts("\r\n");

while( blockCounter < totalBlocks )
{
   i=0;
   do
   {
      data = uart_getc();
      buffer[i++] = data;
     if(i == 512) break;
   }while (data != '~');

   uart_puts("\r\n");
   uart_puts(" ---- ");
   uart_puts("\r\n");

   SPI_transmit(0xfc); //Send start block token 0xfc (0x11111100)

   for(i=0; i<512; i++) //send 512 bytes data
      SPI_transmit(buffer[i]);

   SPI_transmit(0xff); //transmit dummy CRC (16-bit), CRC is ignored here
   SPI_transmit(0xff);

   response = SPI_receive();
   if((response & 0x1f) != 0x05)
   {
      SD_CS_DEASSERT;
      return response;
   }

   while(!SPI_receive())
      if(retry++ > 0xfffe){SD_CS_DEASSERT; return 1;}

   blockCounter++;
}

SPI_transmit(0xfd); //send 'stop transmission token'

retry = 0;

while(!SPI_receive()) //wait for SD card to complete writing and get idle
   if(retry++ > 0xfffe){SD_CS_DEASSERT; return 1;}

SD_CS_DEASSERT;
SPI_transmit(0xff); //just spend 8 clock cycle delay before reasserting the CS signal
SD_CS_ASSERT; //re assertion of the CS signal is required to verify if card is still busy

while(!SPI_receive()) //wait for SD card to complete writing and get idle
   if(retry++ > 0xfffe){SD_CS_DEASSERT; return 1;}
SD_CS_DEASSERT;

return 0;
}
//*********************************************

#endif

//******** END ****** www.dharmanitech.com *****