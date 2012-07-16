#ifndef _EXPANSION_H
#define _EXPANSION_H

#include <math.h>
#include <stdint.h>
#include <include.h>

#define DEBUG 0
#define BOSCH_EEPROM_ADDRESS 0xEE

int16_t eeprom_values[11]; /* Used to store the calibration values read from the Bosch's EEPROM */

// These eeprom register values are from the Bosch datasheet
#define AC1 (int16_t)eeprom_values[0]
#define AC2 (int16_t)eeprom_values[1]
#define AC3 (int16_t)eeprom_values[2]
#define AC4 (uint16_t)eeprom_values[3]
#define AC5 (uint16_t)eeprom_values[4]
#define AC6 (uint16_t)eeprom_values[5]
#define B1 (int16_t)eeprom_values[6]
#define B2 (int16_t)eeprom_values[7]
#define MB (int16_t)eeprom_values[8]
#define MC (int16_t)eeprom_values[9]
#define MD (int16_t)eeprom_values[10]

#define TEMP_BASE_REGISTER 0xF6 /* The address of the initial place to read from for temperature */
#define PRESS_BASE_REGISTER 0xF6 /* The address of the initial place to read from for pressure */

#define ADC_STARTUP_DELAY  1000
#define ADC_SETUP_DELAY  200

int32_t UT, UP; /* The uncompensated temperature and pressure values */

// Temp variables used in converting from the raw values to the finished product
int32_t X1, X2, X3, B3, B5, B6, P, T;
uint32_t B4, B7;

uint32_t h_cnt;
uint8_t channel;
uint8_t is_open;

// VREF is set to VCC by default
#define ADC_INIT() \
    do { \
	ADCSRA = BM(ADPS0) | BM(ADPS1); \
	ADMUX = BM(REFS0);  \
} while (0)

#define ADC_VREF_VCC() \
   do { \
	ADMUX &= ~(BM(REFS1));  \
	ADMUX |= BM(REFS0);  \
} while(0)


#define ADC_VREF_1_1() \
   do { \
	ADMUX &= ~(BM(REFS0));  \
	ADMUX |= BM(REFS1);  \
} while(0)


#define ADC_VREF_2_56() \
   do { \
	ADMUX |= BM(REFS1) | BM(REFS0);  \
} while(0)

#define ADC_SET_CHANNEL(channel) do { ADMUX &= ~0x1F; ADMUX |= (ADMUX & ~0x1F) | (channel); } while (0)

// Enables/disables the ADC
#define ADC_ENABLE() do { ADCSRA |= BM(ADEN); } while (0)
#define ADC_DISABLE() do { ADCSRA &= ~BM(ADEN); } while (0)

#define ADC_SAMPLE_SINGLE() \
    do { \
ADCSRA |= BM(ADSC); \
while (!(ADCSRA & 0x10)); \
} while(0)

// Macros for obtaining the latest sample value
#define ADC_GET_SAMPLE_10(x) \
do { \
x =  ADCL; \
x |= ADCH << 8; \
} while (0)

#define ADC_GET_SAMPLE_8(x) \
do { \
x = ((uint8_t) ADCL) >> 2; \
x |= ((int8_t) ADCH) << 6; \
} while (0)



#define MAX_PAYLOAD 64
#define MAC_SIZE 4
#define TIME_STAMP_SIZE 4



/**
 * Returns 10X the temperature in Celcius. For example, if the temperature is
 * 20.7 degrees Celcius, will return 207. This is the method to call to get the
 * temperature and acts as a wrapper around the other methods.
 *
 * @returns the temperature
 *
 */
int32_t calc_temp();

/**
 * Returns the current pressure in Pascals. Note that standard pressure (1 ATM)
 * is 101,325 Pascals.
 *
 * @param oss - the oversampling mode. Valid values are 0 to 3. Currently the code
 *              appears to be working with all OSS values but the safest is 0.
 *
 * @returns the pressure in pascals
 *
 */
int32_t calc_press(uint8_t oss);


/**
 * Reads the uncompensated pressure value from the Bosch sensor. This method
 * should not be called directly. Instead use the calc_press() is a wrapper and
 * should be used.
 *
 * @param oss - the oversampling mode. Valid values are 0 to 3. Currently the code
 *              appears to be working with all OSS values but the safest is 0.
 *
 * @returns void
 *
 */
void read_uncomp_press(uint8_t oss);

/**
 * Reads the uncompensated temperature value from the Bosch sensor. This method
 * should not be called directly. Instead the calc_temp() is a wrapper and should
 * be used.
 *
 * @returns void
 *
 */
void read_uncomp_temp();

/**
 * Determines the actual compensated temperature value. This method should not be
 * called directly. Instead the calc_temp() is a wrapper and should be used.
 *
 * @returns - the correct temperature value
 *
 */
int32_t calc_true_temp();


/**
 * Determines the actual compensated pressure value.  method should not be called
 * directly. Instead the calc_press() is a wrapper and should be used.
 *
 * @param oss - the oversampling mode. Valid values are 0 to 3. Currently the code
 *              appears to be working with all OSS values but the safest is 0.
 *
 * @reuurns - the correct pressure value
 */
int32_t calc_true_press(uint8_t oss);

/**
 * Reads in the neccessary EEPROM values for the Bosch temperature/pressure
 * sensor. The values are used for compensating the raw values returned from the
 * sensor.
 *
 * @returns - void
 *
 */
void get_eeprom_values(void);

// Functions for initializing and updating sensor values
void init_adc();


typedef struct
{
// Common Header
	uint8_t protocol_id;
	uint8_t priority;
	uint8_t src_mac[MAC_SIZE];
	uint8_t dest_mac[MAC_SIZE];  
	uint8_t seq_num;
	uint8_t ttl;
	uint8_t time_secs[TIME_STAMP_SIZE];
	uint8_t time_nsecs[TIME_STAMP_SIZE];
	uint8_t last_hop[MAC_SIZE];
	uint8_t next_hop[MAC_SIZE];
	uint8_t payload_len;
	uint8_t payload[MAX_PAYLOAD];
	uint8_t checksum;
} drk_packet;


#endif
