/******************************************************************************
*  Nano-RK, a real-time operating system for sensor networks.
*  Copyright (C) 2007, Real-Time and Multimedia Lab, Carnegie Mellon University
*  All rights reserved.
*
*  This is the Open Source Version of Nano-RK included as part of a Dual
*  Licensing Model. If you are unsure which license to use please refer to:
*  http://www.nanork.org/nano-RK/wiki/Licensing
*
*  This program is free software: you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation, version 2.0 of the License.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*
*  You should have received a copy of the GNU General Public License
*  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*******************************************************************************/

#include <nrk.h>
#include <include.h>
#include <ulib.h>
#include <stdio.h>
#include <avr/sleep.h>
#include <hal.h>
#include <bmac.h>
#include <nrk_error.h>
#include <nrk_events.h>
#include <sampl.h>
#include <pkt_packer.h>
#include <aggregate.h>
#include <generate.h>
#include <p2p_handler.h>
#include <globals.h>
#include <nrk_eeprom.h>
#include <route_table.h>
#include <neighbor_list.h>
#include <debug.h>
#include <sampl_tasks.h>
#include <nrk_timer.h>
#include <power_vars.h>
#include <math.h>
#include "transducer_handler.h"
#include <event_detector.h>
#include <eeprom_state.h>
#include <power_vars.h>


#define ADC_SETUP_DELAY  100

uint8_t channel;


#define ADC_INIT() \
    do { \
        ADCSRA = BM(ADPS0) | BM(ADPS1); \
        ADMUX = BM(REFS0); \
} while (0)

#define ADC_SET_CHANNEL(channel) do { ADMUX = (ADMUX & ~0x1F) | (channel); } while (0)

// Enables/disables the ADC
#define ADC_ENABLE() do { ADCSRA |= BM(ADEN); } while (0)
#define ADC_DISABLE() do { ADCSRA &= ~BM(ADEN); } while (0)

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


uint8_t startup_sock_state;

void calc_power();

int main ()
{
  uint8_t t;


  DDRA  = BM(DEBUG_0) | BM(DEBUG_1);
  startup_sock_state=nrk_eeprom_read_byte(0x100);
  if((startup_sock_state&0x1)==0x01) 
  	socket_0_enable();	
  else socket_0_disable();
  if((startup_sock_state&0x2)==0x02) 
  	socket_1_enable();
  else socket_1_disable();
  if((startup_sock_state&0x4)==0x02) 
  	socket_0_push_enabled=1;
  else socket_0_push_enabled=0; 
  if((startup_sock_state&0x8)==0x02) 
  	socket_1_push_enabled=1;
  else socket_1_push_enabled=0; 

//if(nrk_watchdog_check()==NRK_OK)
//{ 
  nrk_setup_ports ();
  nrk_setup_uart (UART_BAUDRATE_115K2);
//}
  nrk_init ();

  nrk_led_clr (0);
  nrk_led_clr (1);
  nrk_led_clr (2);
  nrk_led_clr (3);


  t=nrk_eeprom_read_byte(0x101);
  if(t==0) { nrk_eeprom_write_byte(0x101,1); }
  else { nrk_eeprom_write_byte(0x101,0); }
  

  nrk_time_set (0, 0);

  bmac_task_config ();
  sampl_config();
  nrk_create_taskset ();
  nrk_start ();

  return 0;
}


void nrk_create_taskset ()
{
uint8_t t;
  ADC_INIT ();
  ADC_ENABLE ();
  ADC_VREF_VCC(); 
  //nrk_timer_int_configure( NRK_APP_TIMER_0, 5, 7, calc_power);
  nrk_timer_int_configure( NRK_APP_TIMER_0, 1, 7373, calc_power);
  ticks=0;
  cycle_state=CYCLE_HIGH;
  //cycle_state_last=CYCLE_UNKNOWN;
  cycle_cnt=0;
  cycle_avg=0;
  cycle_started=0;
  c1_center=496;
  v_p2p_low=2000;
  v_p2p_high=0;
  c_p2p_low=2000;
  c_p2p_high=0;
  c_p2p_low2=2000;
  c_p2p_high2=0;
  rms_current=0;
  rms_current2=0;
  rms_voltage=0;
  energy_total2=0;
  energy_total=0;
  energy_cycle2=0;
  energy_cycle=0;
 
  if(nrk_eeprom_read_byte(EEPROM_MAGIC)==0x42)
	{
	// find largest energy and load
  	energy_eeprom_read();
	}
   else
	{
	// first time running, clear area 
	energy_eeprom_erase();
	}

  total_secs=0;
  v_last=VOLTAGE_LOW_THRESHOLD+10;
  triggered=0;

  nrk_gpio_direction(NRK_DEBUG_2, NRK_PIN_OUTPUT);

  socket_0_push_threshold=nrk_eeprom_read_byte(EEPROM_PUSH_THRESHOLD_0);
  socket_1_push_threshold=nrk_eeprom_read_byte(EEPROM_PUSH_THRESHOLD_1);
  startup_sock_state=nrk_eeprom_read_byte(EEPROM_SOCKET_STATE);
  if((startup_sock_state&0x04)==0x04) socket_0_push_enabled=1;
  else socket_0_push_enabled=0; 
  if((startup_sock_state&0x08)==0x08) socket_1_push_enabled=1;
  else socket_1_push_enabled=0; 
  if((startup_sock_state&0x01)==0x01) 
	{
	 	nrk_timer_int_start(NRK_APP_TIMER_0);
                power_mon_enable();
                socket_0_enable();
                socket_0_active=1;
		//nrk_kprintf( PSTR("Socket active\r\n"));
	}
  else {
  socket_0_active=0;
  power_mon_disable();
		//nrk_kprintf( PSTR("Socket inactive\r\n"));
  }
  if((startup_sock_state&0x02)==0x02) 
	{
	 	nrk_timer_int_start(NRK_APP_TIMER_0);
                socket_1_enable();
                socket_1_active=1;
	}
  else {
  socket_1_active=0;
  }

  nrk_task_set_entry_function( &event_detector, event_detector_task);
  nrk_task_set_stk( &event_detector, event_detector_stack, EVENT_DETECTOR_STACKSIZE);
  event_detector.prio = 1;
  event_detector.FirstActivation = TRUE;
  event_detector.Type = BASIC_TASK;
  event_detector.SchType = PREEMPTIVE;
  event_detector.period.secs = 1;
  event_detector.period.nano_secs = 0*NANOS_PER_MS;
  event_detector.cpu_reserve.secs = 0;
  event_detector.cpu_reserve.nano_secs = 500*NANOS_PER_MS;
  event_detector.offset.secs = 0;
  event_detector.offset.nano_secs= 0;
  nrk_activate_task (&event_detector);

}


void calc_power()
{
    // FIXME: Disable interrupt later if power off to save energy
//    if(socket_0_active==0 && socket_1_active==0) return;
 //  nrk_int_disable();

#ifdef CT_METER
    ADC_SET_CHANNEL (1);
    nrk_spin_wait_us(ADC_SETUP_DELAY);
    ADC_SAMPLE_SINGLE();
    ADC_GET_SAMPLE_10(v);

    ADC_SET_CHANNEL (2);
    nrk_spin_wait_us(ADC_SETUP_DELAY);
    ADC_SAMPLE_SINGLE();
    ADC_GET_SAMPLE_10(c1);

    ADC_SET_CHANNEL (3);
    nrk_spin_wait_us(ADC_SETUP_DELAY);
    ADC_SAMPLE_SINGLE();
    ADC_GET_SAMPLE_10(c2);

    ADC_SET_CHANNEL (5);
    nrk_spin_wait_us(ADC_SETUP_DELAY);
    ADC_SAMPLE_SINGLE();
    ADC_GET_SAMPLE_10(center_chan);
    c1_center=(uint16_t)center_chan;
#else
    ADC_SET_CHANNEL (4);
    nrk_spin_wait_us(ADC_SETUP_DELAY);
    ADC_SAMPLE_SINGLE();
    ADC_GET_SAMPLE_10(v);

    ADC_SET_CHANNEL (5);
    nrk_spin_wait_us(ADC_SETUP_DELAY);
    ADC_SAMPLE_SINGLE();
    ADC_GET_SAMPLE_10(c1);

    ADC_SET_CHANNEL (6);
    //nrk_spin_wait_us(ADC_SETUP_DELAY);
    ADC_SAMPLE_SINGLE();
    ADC_GET_SAMPLE_10(c2);

    ADC_SET_CHANNEL (7);
    nrk_spin_wait_us(ADC_SETUP_DELAY);
    ADC_SAMPLE_SINGLE();
    ADC_GET_SAMPLE_10(center_chan);

    c1_center=(uint16_t)(((uint32_t)center_chan*647)/1000);
#endif

// Catch the rising edge after the voltage dips below a low threshold
// Then ignore that case until the voltage goes up again.
// This filter is used to detect the zero crossing point
if(triggered==0 && v<VOLTAGE_LOW_THRESHOLD && v>v_last)
{
if(cycle_state==CYCLE_HIGH) cycle_state=CYCLE_LOW;
else { 
	// Low to High transition
	cycle_state=CYCLE_HIGH; 
	cycle_cnt++; 
	if(energy_cycle<0) energy_cycle*=-1;
	energy_total+=energy_cycle;
	if(energy_cycle2<0) energy_cycle2*=-1;
	energy_total2+=energy_cycle2;
	energy_cycle=0;
	energy_cycle2=0;
	}
cycle_started=1;
triggered=1;
}

// Reset filter trap after the voltage goes up
if(v>VOLTAGE_LOW_THRESHOLD) triggered=0;
v_last=v;

    if(cycle_started==1) 
	{
	ticks++;
	if(c1<c_p2p_low) c_p2p_low=c1;
	if(c1>c_p2p_high) c_p2p_high=c1;
	if(v<v_p2p_low) v_p2p_low=v;
	if(v>v_p2p_high) v_p2p_high=v;
	c1-=c1_center;

	if(c2<c_p2p_low2) c_p2p_low2=c2;
	if(c2>c_p2p_high2) c_p2p_high2=c2;
	c2-=c1_center;
	

	// remove noise at floor
	//if(c1<10) c1=0;
	
	current_total+=((int32_t)c1*(int32_t)c1);
	voltage_total+=((int32_t)v*(int32_t)v);
	if(cycle_state==CYCLE_LOW) v*=-1;
	energy_cycle+=((int32_t)c1*(int32_t)v);
	
	current_total2+=((int32_t)c2*(int32_t)c2);
	energy_cycle2+=((int32_t)c2*(int32_t)v);
	//printf( "c1=%u current=%lu\r\n",c1, current_total );

	
	if(ticks>=1000) 
	{

	// Save values to pass to event detector functions that calculate power for
	// packets etc
	freq=cycle_cnt;
	l_v_p2p_high=v_p2p_high;
  	l_v_p2p_low=v_p2p_low;
  	l_c_p2p_high=c_p2p_high;
  	l_c_p2p_low=c_p2p_low;
  	l_c_p2p_high2=c_p2p_high2;
  	l_c_p2p_low2=c_p2p_low2;
	ticks_last=ticks;
	current_total_last=current_total;
	current_total2_last=current_total2;
	energy_total_last=energy_total;
	energy_total2_last=energy_total2;
	voltage_total_last=voltage_total;


	// Reset values for next cycle	
  	ticks=0;
  	cycle_cnt=0;
  	voltage_total=0;
  	energy_total=0;
  	energy_total2=0;
  	current_total=0;
  	current_total2=0;
  	v_p2p_low=2000;
  	v_p2p_high=0;
  	c_p2p_low=2000;
  	c_p2p_high=0;
  	c_p2p_low2=2000;
  	c_p2p_high2=0;

	// Signal event detector task
	nrk_event_signal(update_energy_sig);
	}


	}

//nrk_int_enable();
}
void sampl_startup()
{
write_eeprom_flag=0;

}
