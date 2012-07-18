#ifndef _JIGA_CAL_H_
#define _JIGA_CAL_H_

#define MAX_JIGA_CAL_SIZE	24	

#include <stdint.h>


typedef struct jiga_watt_cal_struct
{

   uint32_t mac;
   uint8_t outlet;
   int16_t current_adc_offset;
   float voltage_scaler;
   float current_scaler;
   float power_scaler;
} jiga_watt_cal_struct_t;


void jiga_watt_cal_load_params(char *file_name);
int jiga_cal_get(uint32_t mac, uint8_t outlet, jiga_watt_cal_struct_t *j);



#endif
