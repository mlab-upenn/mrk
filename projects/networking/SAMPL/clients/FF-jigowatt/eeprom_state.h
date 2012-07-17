#ifndef EEPROM_STATE_H_
#define EEPROM_STATE_H_

#include <stdint.h>

#define EEPROM_PUSH_THRESHOLD_0	0x098
#define EEPROM_PUSH_THRESHOLD_1	0x099
#define EEPROM_SOCKET_STATE	0x100
#define EEPROM_MAGIC		0x102
#define EEPROM_ENERGY_START	0x104
#define EEPROM_ENERGY_SIZE	((uint16_t)17)	
#define EEPROM_LEVELS		((uint16_t)0x08)


uint16_t energy_eeprom_addr_index;
uint8_t energy_eeprom_cnt;
uint8_t write_eeprom_flag;

void energy_eeprom_read(void);
void energy_eeprom_erase();
void energy_eeprom_write(void);



#endif
