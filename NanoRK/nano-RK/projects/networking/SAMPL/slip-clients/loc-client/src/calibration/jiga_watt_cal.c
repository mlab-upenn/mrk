#include <stdio.h>
#include <stdlib.h>
#include <jiga_watt_cal.h>
#include <math.h>

static int cal_elements;
static jiga_watt_cal_struct_t cal_params[MAX_JIGA_CAL_SIZE];

void jiga_watt_cal_load_params (char *file_name)
{
  int i, v;
  FILE *fp;
  char buf[1024];
  uint32_t t_mac;
  uint32_t t_outlet;
  uint32_t t_current_adc_offset;
  float t_voltage_scaler, t_current_scaler, t_power_scaler;

  cal_elements = 0;
  for (i = 0; i < MAX_JIGA_CAL_SIZE; i++)
    cal_params[i].mac = 0;
  fp = fopen (file_name, "r");
  if (fp == NULL) {
    printf ("Could not open calibration file: %s\n", file_name);
    return;
  }

  do {
    v = fscanf (fp, "%[^\n]\n", buf);
    if (v != -1 && buf[0] != '#') {
      v =
        sscanf (buf, "%x %d %d %f %f %f", &t_mac, &t_outlet, &t_current_adc_offset,
                &t_voltage_scaler, &t_current_scaler, &t_power_scaler);
      if (v == 6) {
        cal_params[cal_elements].mac = t_mac;
        cal_params[cal_elements].outlet = t_outlet;
        cal_params[cal_elements].current_adc_offset =
          (uint16_t) t_current_adc_offset;
        cal_params[cal_elements].voltage_scaler = t_voltage_scaler;
        cal_params[cal_elements].current_scaler = t_current_scaler;
        cal_params[cal_elements].power_scaler = t_power_scaler;
        printf
          ("jiga cal mac 0x%08x outlet=%d current_adc_offset=%d v-scaler=%f i-scaler=%f p-sclaer=%f\n",
           cal_params[cal_elements].mac,
           cal_params[cal_elements].outlet,
           cal_params[cal_elements].current_adc_offset,
           cal_params[cal_elements].voltage_scaler,
           cal_params[cal_elements].current_scaler,
           cal_params[cal_elements].power_scaler);
        if (cal_elements < MAX_JIGA_CAL_SIZE)
          cal_elements++;
      }
    }
  } while (v != -1);

  fclose (fp);
}

int jiga_watt_cal_get (uint32_t mac, uint8_t outlet, jiga_watt_cal_struct_t * j)
{
  int i;


  for (i = 0; i < MAX_JIGA_CAL_SIZE; i++)
    if (cal_params[i].mac == mac && cal_params[i].outlet == outlet) {
      j->mac = mac;
      j->current_adc_offset = cal_params[i].current_adc_offset;
      j->voltage_scaler = cal_params[i].voltage_scaler;
      j->current_scaler = cal_params[i].current_scaler;
      j->power_scaler = cal_params[i].power_scaler;
      return 1;
    }
  return 0;
}
