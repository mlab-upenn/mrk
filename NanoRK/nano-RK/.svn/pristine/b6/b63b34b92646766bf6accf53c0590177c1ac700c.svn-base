#ifndef I2CSW_H
#define I2CSW_H

// defines and constants
#define READ        0x01    // I2C READ bit

// functions

// initialize I2C interface pins
void i2cInit(void);

// send I2C data to <device> register <sub>
int8_t i2cSend(uint8_t device, uint8_t  sub, uint8_t length, uint8_t *data);

// receive I2C data from <device> register <sub>
int8_t i2cReceive(uint8_t  device, uint8_t  sub, uint8_t  length, uint8_t  *data);

#endif
