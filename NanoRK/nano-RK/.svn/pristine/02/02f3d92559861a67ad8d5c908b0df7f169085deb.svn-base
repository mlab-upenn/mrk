#include <globals.h>
#include <nrk.h>
#include <sampl.h>
#include <transducer_pkt.h>


uint8_t g_binary_sensor_value;
uint8_t g_binary_sensor_type;

typedef struct two_byte_adc_raw 
{
	uint8_t chan;        
	uint16_t value;        
} TWO_BYTE_ADC_RAW_T;


int8_t transducer_handler(TRANSDUCER_MSG_T *in_msg, TRANSDUCER_MSG_T *out_msg);
