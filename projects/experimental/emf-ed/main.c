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
#include <nrk_error.h>
#include <nrk_timer.h>
//#include <nrk_driver_list.h>
//#include <nrk_driver.h>
//#include <adc_driver.h>


#define ADC_SETUP_DELAY  500

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





NRK_STK Stack1[NRK_APP_STACKSIZE];
nrk_task_type TaskOne;
void Task1(void);


void nrk_create_taskset();
void nrk_register_drivers();
uint8_t kill_stack(uint8_t val);

double tmp_d;
uint32_t current_total2, c2,c1;

	int
main ()
{
  uint8_t t;
  nrk_setup_ports();
  nrk_setup_uart(UART_BAUDRATE_230K4);

  nrk_init();
  nrk_time_set(0,0);

  nrk_create_taskset ();
  nrk_start();
  
  return 0;
}


void Task1()
{
uint16_t cnt;
int8_t fd,val,chan;
uint16_t sample;


   printf( "Task1 PID=%d\r\n",nrk_get_pid());

  nrk_gpio_direction(NRK_BUTTON, NRK_PIN_INPUT);
  nrk_gpio_direction(NRK_DEBUG_0, NRK_PIN_OUTPUT);

  nrk_led_set(RED_LED);
  do{} while(nrk_gpio_get(NRK_BUTTON)==1);
  nrk_led_clr(RED_LED);
  nrk_led_set(GREEN_LED);

   // Initialize values here
   ADC_INIT ();
   ADC_ENABLE ();
   ADC_SET_CHANNEL (2);

  while(1) {
	ADC_SAMPLE_SINGLE();
	ADC_GET_SAMPLE_10(sample);
  	// Send sync byte
  	putchar(0x55);
	putchar(sample>>8);
	putchar(sample&0xff);
  	}
}


void
nrk_create_taskset()
{
  TaskOne.task = Task1;
  nrk_task_set_stk( &TaskOne, Stack1, NRK_APP_STACKSIZE);
  TaskOne.prio = 1;
  TaskOne.FirstActivation = TRUE;
  TaskOne.Type = BASIC_TASK;
  TaskOne.SchType = PREEMPTIVE;
  TaskOne.period.secs = 0;
  TaskOne.period.nano_secs = 5*NANOS_PER_MS;
  // Disable reserve
  TaskOne.cpu_reserve.secs = 0;
  TaskOne.cpu_reserve.nano_secs =  0;
  TaskOne.offset.secs = 0;
  TaskOne.offset.nano_secs= 0;
  nrk_activate_task (&TaskOne);

}


