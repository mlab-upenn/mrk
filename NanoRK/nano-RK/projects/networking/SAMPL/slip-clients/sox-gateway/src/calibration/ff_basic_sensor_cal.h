#ifndef _SENSOR_CAL_H_
#define _SENSOR_CAL_H_

#define MAX_CAL_SIZE	64

#include <stdint.h>


typedef struct ff_basic_sensor_cal_struct
{

   uint32_t mac;
   float temp_offset;
   int16_t light_offset;
} ff_basic_sensor_cal_struct_t;


void ff_basic_sensor_cal_load_params(char *file_name);
float ff_basic_sensor_cal_get_temp(uint16_t adc_in );
uint16_t ff_basic_sensor_cal_get_light(uint16_t adc_in, float voltage);
float ff_basic_sensor_cal_get_temp_offset( uint32_t mac );
int16_t ff_basic_sensor_cal_get_light_offset( uint32_t mac );


#endif
