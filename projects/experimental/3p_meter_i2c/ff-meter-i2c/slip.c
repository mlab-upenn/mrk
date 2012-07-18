#include <avr/interrupt.h>
#include <stdio.h>
#include "slip.h"

uint8_t slip_tx (uint8_t * buf, int size, uint8_t * EndOfBuff,
                 uint32_t MaxSizeBuff, uint8_t * buf_out)
{
  uint8_t i;
  uint8_t checksum;
  int8_t c;
  uint8_t index = 0;

  if (size > 128)
    return;

  checksum = 0;

  //Send Start byte
  c = STARTx;
  *(buf_out + index++) = c;     //putchar(c);

  c = size;
  *(buf_out + index++) = c;     //putchar(c);;

  for (i = 0; i < size; i++) {
    if (*buf == END || *buf == ESC) {
      c = ESC;
      *(buf_out + index++) = c; //putchar(c);;
    }
    c = *buf;
    *(buf_out + index++) = c;   //putchar(c);;
    checksum += *buf;

    buf++;
    if (buf > EndOfBuff)
      buf = buf - MaxSizeBuff;
  }

  checksum &= 0x7f;

  //Send end byte
  c = checksum;
  *(buf_out + index++) = c;     //putchar(c);;
  c = END;
  *(buf_out + index++) = c;     //putchar(c);;

  return index;
}

/*
void slip_tx (uint8_t *buf, int size, uint8_t *EndOfBuff, uint32_t MaxSizeBuff)
{
	uint8_t i;
	uint8_t checksum;
	int8_t c;

	if(size > 128)
		return;

	checksum = 0;

	//Send Start byte
	c = STARTx;
	putchar(c);

	c = size;
	putchar(c);

	for (i=0; i<size; i++)
	{
		if(*buf == END || *buf ==ESC)
		{
			c = ESC;
			putchar(c);
		}
		c = *buf;
		putchar(c);
		checksum += *buf;

		buf++;
		if(buf > EndOfBuff)
			buf = buf-MaxSizeBuff;
	}

	checksum &= 0x7f;

	//Send end byte
	c = checksum;
	putchar(c);
	c = END;
	putchar(c);
}*/
