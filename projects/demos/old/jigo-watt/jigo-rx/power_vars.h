#ifndef _POWER_VARS_H_
#define _POWER_VARS_H_

#define VOLTAGE_LOW_THRESHOLD	500	
#define CYCLE_UNKNOWN	0
#define CYCLE_LOW	1
#define CYCLE_HIGH	2

uint16_t center_chan;
int16_t v,v_last;
int16_t c1,c2;
uint16_t l_v_p2p_low,l_v_p2p_high, l_c_p2p_low, l_c_p2p_high;
uint16_t v_p2p_low,v_p2p_high, c_p2p_low, c_p2p_high;
uint8_t socket_0_active;
uint8_t socket_1_active;
uint16_t c1_center;
uint16_t rms_current;
uint16_t rms_voltage;
uint32_t true_power;
double cummulative_energy;
uint32_t total_secs;
uint32_t tmp_energy;

uint16_t ticks;
uint8_t cycle_state;
uint8_t cycle_state_last;
uint16_t cycle_cnt;
uint16_t cycle_avg;
uint16_t freq;
uint8_t cycle_started;

int32_t energy_cycle;
uint32_t energy_total;
uint32_t current_total;
uint16_t current_avg;
double tmp_d;

uint32_t voltage_total;
uint16_t voltage_avg;

uint32_t freq_total;
uint16_t freq_avg;
uint16_t freq_cnt;

uint8_t triggered;

#endif
