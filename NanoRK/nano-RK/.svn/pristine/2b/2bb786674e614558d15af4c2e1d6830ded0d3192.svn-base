#include <avr/io.h>
#include <avr/interrupt.h>
#include "ade7878.h"

#include <nrk.h>
  
// Add this #define if using the i2c adapter board. 
// This will swap the interrupt pin so that i2c is free
#define I2C_ADAPTER_INSTALLED
  
//This external interrupt is sent by the 7878 when we reset it
#ifndef I2C_ADAPTER_INSTALLED
  ISR (INT1_vect) 
#else   
  ISR (INT3_vect) 
#endif  
{
  uint8_t i;
  EIMSK = 0x00;
  
    //Set MOSI and SCK as output, others as input
    DDRB |= _BV (PORTB0) | _BV (PORTB1) | _BV (PORTB2);
  DDRB &= ~(_BV (PORTB3));
  PORTB |= _BV (PORTB0) | _BV (PORTB1) | _BV (PORTB2);
  for (i = 0; i < 50; i++)
     {
    PORTB ^= _BV (PORTB0);     //We need to toggle this multiple times (>3) to set the SPI mode of communication
    }
  F_ChkIntEntry_U8R = 1;
}


// This function does a few tasks before the OS starts that are timing critical at powerup
inline void ade_nrk_setup () 
{
  DDRD |= _BV (PORTD3);     //ADE7878 Reset line 
  
    //Interrupt initialization for ADE7878
    sei ();                     //Enable Global interrupts
#ifndef I2C_ADAPTER_INSTALLED
  DDRD &= ~(_BV (PORTD1));      //Making INT1 pin as input
#else   /*  */
  DDRD &= ~(_BV (PORTD3));      //Making INT3 pin as input
#endif  /*  */
  EIMSK = 0x00;                //Mask all
  EICRA = 0x08;                 //EINT1 Falling Edge - changed for new FF
  PORTD |= _BV (PORTD3);       //Reset ADE7878
  //Set ADE in normal power mode PM0=1 (PE2), PM1=0 (PE3)
  //nrk_kprintf(PSTR("delay11111111111111111111111111")); //delay - may not be needed now, was added when I was debugging some other issue
  PORTD &= ~_BV (PORTD3);
  
    //Set ADE in normal power mode PM0=1 (PE2), PM1=0 (PE3)
    DDRE |= _BV (PORTE2) | _BV (PORTE3);
  PORTE |= _BV (PORTE2);
  PORTE &= ~(_BV (PORTE3));
  
    //These bunch of lines below may not be needed since they have- added it while debugging a recent issue, did not try the code without these lines
#ifndef I2C_ADAPTER_INSTALLED
    EIMSK = 0x02;               //Unmask EINT1
#else   
    EIMSK = 0x08;               //Unmask EINT3
#endif  
//      nrk_kprintf(PSTR("delay22222222222222222222222222"));
  PORTD |= _BV (PORTD3);
  PORTE |= _BV (PORTE2);
  PORTE &= ~(_BV (PORTE3));
  
    //Check if the interrupt really executed or not
    while (EIMSK != 0);
  
    //Enable SPI Master, Set Clock rate fck/4
    SPCR = 0x01;
  SPCR |= _BV (SPE) | _BV (MSTR) | _BV (CPOL) | _BV (CPHA);
  PORTB |= _BV (PORTB0);      //default state of SS should be low
  
    //This segment below is added because it was observed that the 7878
    //SPI or the ATMEL SPI takes some time to start. So waiting till
    //some register gives its default value
    nrk_kprintf (PSTR ("Wait.."));
  V_RdLcycmode_U8R = 0;
  while (V_RdLcycmode_U8R != 0x78)
    V_RdLcycmode_U8R = ade_read8 (LCYCMODE);
  printf ("\r\n7878 Ready %x", V_RdLcycmode_U8R);
  
    //Initialize ADE7878 DSP and a few other registers
    if (ade_init () < 0)
    nrk_kprintf (PSTR ("\nInit failed"));
  
  else
    nrk_kprintf (PSTR ("\nInit Success"));
  
    //Read of this register - just for checking
    V_RdLcycmode_U8R = ade_read8 (LCYCMODE);
  
    //printf("\r\nLCYC =  %x", V_RdLcycmode_U8R);
    
    //Read the INT Status register to clear the interrupt
    V_Status1RdWr_U32R = ade_read32 (STATUS1);
  ade_write32 (STATUS1, V_Status1RdWr_U32R);
}

int8_t ade_init (void)
{
  int8_t ret;
  
    // Lock SPI mode
    ret = ade_write8 (CONFIG2, 0x0);
  if (ret < 0)
    return -1;
  
    // Pair current inputs with matching voltage inputs
    ret = ade_write16 (CONFIG, VTOIA_A | VTOIB_B | VTOIC_C);
  if (ret < 0)
    return -1;
  
    // Calibrate for -7° phase error
    ret = ade_write16 (CPHCAL, 260);
  if (ret < 0)
    return -1;
  
    // Activate DSP
    ret = ade_write16 (RUN, START);
  if (ret < 0)
    return -1;
  return 0;
}

uint8_t ade_read8 (uint16_t addr)
{
  uint8_t data;
  
    // Turn on RX LED
    PORTD &= ~(_BV (PORTD6));
  
    // Initiate communication
    PORTB &= ~(_BV (PORTB0));
  SPDR = 0x01;
  while (!(SPSR & _BV (SPIF)));
  
    // Send register address to be read 
    SPDR = addr >> 0x8;
  while (!(SPSR & _BV (SPIF)));
  SPDR = addr;
  while (!(SPSR & _BV (SPIF)));
  
    // Receive data
    SPDR = 0x00;
  while (!(SPSR & (1 << SPIF)));
  data = SPDR;
  
    // Stop communication and turn off RX LED
    PORTB |= _BV (PORTB0);
  PORTD |= _BV (PORTD6);
  return data;
}

uint16_t ade_read16 (uint16_t addr)
{
  uint8_t i;
  uint16_t read;
  
    // Turn on RX LED
    PORTD &= ~(_BV (PORTD6));
  
    // Initiate communication
    PORTB &= ~(_BV (PORTB0));
  SPDR = 0x01;
  while (!(SPSR & _BV (SPIF)));
  
    // Send register address to be read 
    SPDR = addr >> 0x8;
  while (!(SPSR & _BV (SPIF)));
  SPDR = addr;
  while (!(SPSR & _BV (SPIF)));
  
    // Receive data
    for (i = 0; i < 2; i++) {
    SPDR = 0x00;
    while (!(SPSR & _BV (SPIF)));
    read = read << 0x8;
    read |= SPDR;
  }
  
    // Stop communication and turn off RX LED
    PORTB |= _BV (PORTB0);
  PORTD |= _BV (PORTD6);
  return read;
}

uint32_t ade_read32 (uint16_t addr)
{
  uint8_t i;
  uint32_t data;
  
    // Turn on RX LED
    PORTD &= ~(_BV (PORTD6));
  
    // Initiate communication
    PORTB &= ~(_BV (PORTB0));
  SPDR = 0x01;
  while (!(SPSR & _BV (SPIF)));
  
    // Send register address to be read 
    SPDR = addr >> 0x8;
  while (!(SPSR & _BV (SPIF)));
  SPDR = addr;
  while (!(SPSR & _BV (SPIF)));
  
    // Receive data
    for (i = 0; i < 4; i++) {
    SPDR = 0x00;
    while (!(SPSR & _BV (SPIF)));
    data = data << 0x8;
    data |= SPDR;
  }
  
    // Stop communication and turn off RX LED
    PORTB |= _BV (PORTB0);
  PORTD |= _BV (PORTD6);
  return data;
}

int8_t ade_write8 (uint16_t addr, uint8_t data)
{
  
    // Turn on TX LED
    PORTD &= ~(_BV (PORTD7));
  
    // Initiate communication
    PORTB &= ~(_BV (PORTB0));
  SPDR = 0x00;
  while (!(SPSR & _BV (SPIF)));
  
    // Send register address to be written to 
    SPDR = addr >> 0x8;
  while (!(SPSR & _BV (SPIF)));
  SPDR = addr;
  while (!(SPSR & _BV (SPIF)));
  
    // Send data
    SPDR = data;
  while (!(SPSR & _BV (SPIF)));
  
    // Stop communication and turn off TX LED
    PORTB |= _BV (PORTB0);
  PORTD |= _BV (PORTD7);
  
    // Check what was just written
    if (ade_read8 (addr) == data)
    return 0;
  
  else
    return -1;
}

int8_t ade_write16 (uint16_t addr, uint16_t data)
{
  int8_t i;
  
    // Turn on TX LED
    PORTD &= ~(_BV (PORTD7));
  
    // Initiate communication
    PORTB &= ~(_BV (PORTB0));
  SPDR = 0x00;
  while (!(SPSR & _BV (SPIF)));
  
    // Send register address to be written to 
    SPDR = addr >> 0x8;
  while (!(SPSR & _BV (SPIF)));
  SPDR = addr;
  while (!(SPSR & _BV (SPIF)));
  
    // Send data
    for (i = 8; i >= 0; i -= 8) {
    SPDR = data >> i;
    while (!(SPSR & _BV (SPIF)));
  }
  
    // Stop communication and turn off TX LED
    PORTB |= _BV (PORTB0);
  PORTD |= _BV (PORTD7);
  
    // Check what was just written
    if (ade_read16 (addr) == data)
    return 0;
  
  else
    return -1;
}

int8_t ade_write32 (uint16_t addr, uint32_t data)
{
  int8_t i;
  
    // Turn on TX LED
    PORTD &= ~(_BV (PORTD7));
  
    // Initiate communication
    PORTB &= ~(_BV (PORTB0));
  SPDR = 0x00;
  while (!(SPSR & _BV (SPIF)));
  
    // Send register address to be written to 
    SPDR = addr >> 0x8;
  while (!(SPSR & _BV (SPIF)));
  SPDR = addr;
  while (!(SPSR & _BV (SPIF)));
  
    // Send data
    for (i = 24; i >= 0; i -= 8) {
    SPDR = data >> i;
    while (!(SPSR & _BV (SPIF)));
  }
  
    // Stop communication and turn off TX LED
    PORTB |= _BV (PORTB0);
  PORTD |= _BV (PORTD7);
  
    // Check what was just written
    if (ade_read32 (addr) == data)
    return 0;
  
  else
    return -1;
}

int8_t ade_set8 (uint16_t addr, uint8_t data)
{
  uint8_t current = ade_read8 (addr);
  current |= data;
  return ade_write8 (addr, current);
}

int8_t ade_set16 (uint16_t addr, uint16_t data)
{
  uint16_t current = ade_read16 (addr);
  current |= data;
  return ade_write16 (addr, current);
}

int8_t ade_set32 (uint16_t addr, uint32_t data)
{
  uint32_t current = ade_read32 (addr);
  current |= data;
  return ade_write32 (addr, current);
}

int8_t ade_clear8 (uint16_t addr, uint8_t data)
{
  uint8_t current = ade_read8 (addr);
  current &= ~data;
  return ade_write8 (addr, current);
}

int8_t ade_clear16 (uint16_t addr, uint16_t data)
{
  uint16_t current = ade_read16 (addr);
  current &= ~data;
  return ade_write16 (addr, current);
}

int8_t ade_clear32 (uint16_t addr, uint32_t data)
{
  uint32_t current = ade_read32 (addr);
  current &= ~data;
  return ade_write32 (addr, current);
}


