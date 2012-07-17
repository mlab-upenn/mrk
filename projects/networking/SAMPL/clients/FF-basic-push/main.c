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
#include <nrk_driver_list.h>
#include <nrk_driver.h>
#include <ff_basic_sensor.h>
#include <nrk_eeprom.h>
#include <route_table.h>
#include <neighbor_list.h>
#include <debug.h>
#include <sampl_tasks.h>
#include <transducer_registry.h>
#include <transducer_pkt.h>
#include <nrk_ext_int.h>

uint8_t async_sensor_pkt[64];
uint8_t async_sensor;
#define LIQUID_DETECTOR	1

static TRANSDUCER_MSG_T tran_msg;
static TRANSDUCER_PKT_T tran_pkt;

nrk_task_type ASYNC_SENSING_TASK;
NRK_STK async_sensing_task_stack[NRK_APP_STACKSIZE];
void async_sensing_task (void);

nrk_sig_t async_sig;

int main ()
{
  nrk_setup_ports ();
  nrk_setup_uart (UART_BAUDRATE_115K2);


  nrk_init ();

  nrk_led_clr (0);
  nrk_led_clr (1);
  nrk_led_clr (2);
  nrk_led_clr (3);

  nrk_time_set (0, 0);

  bmac_task_config ();
  sampl_config();
  nrk_register_drivers();


  nrk_create_taskset ();
  nrk_start ();

  return 0;
}

void nrk_register_drivers()
{
int8_t val;

// Register the Basic FireFly Sensor device driver
// Make sure to add: 
//     #define NRK_MAX_DRIVER_CNT  
//     in nrk_cfg.h
// Make sure to add: 
//     SRC += $(ROOT_DIR)/src/drivers/platform/$(PLATFORM_TYPE)/source/ff_basic_sensor.c
//     in makefile
val=nrk_register_driver( &dev_manager_ff_sensors,FIREFLY_SENSOR_BASIC);
if(val==NRK_ERROR) nrk_kprintf( PSTR("Failed to load my ADC driver\r\n") );

}

void leak_detect()
{
int8_t v;
   v=nrk_event_signal( async_sig);
   if(v==NRK_ERROR) nrk_kprintf( PSTR( "nrk_event_signal failed\r\n" ));


}

void sampl_startup()
{


}

void async_sensing_task()
{
  nrk_sig_mask_t my_sigs;
  uint8_t i,len;
  int8_t v;

  async_sig=nrk_signal_create();
  if(async_sig==NRK_ERROR)
	{
	nrk_kprintf( PSTR( "Error allocating async signal!\r\n" ));
	nrk_led_set(RED_LED);
	}

  v=nrk_signal_register(async_sig);
  if(v==NRK_ERROR) nrk_kprintf( PSTR( "Error calling nrk_signal_register\r\n" ));

 nrk_gpio_direction(NRK_DEBUG_2 , NRK_PIN_INPUT);
 i=nrk_ext_int_configure( NRK_EXT_INT_0,NRK_LEVEL_TRIGGER, &leak_detect ); 
 nrk_ext_int_enable ( NRK_EXT_INT_0);


while(1)
 {
   my_sigs=nrk_event_wait( SIG(async_sig)  );

  if(my_sigs==0) nrk_kprintf( PSTR( "nrk_event_wait failed\r\n" ));
  if(my_sigs & SIG(async_sig))
	{
 	// wait on signal from interrupt
 	//TRANSDUCER_PKT_T r;
 	//r.mac_addr = my_mac;
 	//r.len = 0;
 	//r.type = TRAN_LIQUID_SENSOR;

  tran_pkt.num_msgs=0;
  tran_pkt.checksum=0;
  tran_pkt.msgs_payload=&(async_sensor_pkt[TRANSDUCER_PKT_HEADER_SIZE]);
  tran_msg.payload=&(async_sensor_pkt[TRANSDUCER_PKT_HEADER_SIZE+TRANSDUCER_MSG_HEADER_SIZE]);
  tran_msg.type=TRAN_BINARY_SENSOR;
  tran_msg.mac_addr=my_mac;
  tran_msg.payload[0]=(1<<7) | BINARY_SENSOR_LEAK;
  tran_msg.len=1;	
  len=transducer_msg_add( &tran_pkt, &tran_msg);
  len = transducer_pkt_pack(&tran_pkt, async_sensor_pkt);

	for(i=0; i<2; i++ )
		{
 		// push_p2p_pkt( payload, size, pkt_type, dst_mac );
 		push_p2p_pkt( async_sensor_pkt,len,TRANSDUCER_PKT, 0x0 );
 		nrk_led_toggle(RED_LED);
		nrk_wait_until_next_period();
		}
 		nrk_led_clr(RED_LED);
	}
 }

}


void nrk_create_taskset ()
{
  ASYNC_SENSING_TASK.task = async_sensing_task;
  nrk_task_set_stk (&ASYNC_SENSING_TASK, async_sensing_task_stack, NRK_APP_STACKSIZE);
  ASYNC_SENSING_TASK.prio = 5;
  ASYNC_SENSING_TASK.FirstActivation = TRUE;
  ASYNC_SENSING_TASK.Type = BASIC_TASK;
  ASYNC_SENSING_TASK.SchType = PREEMPTIVE;
  ASYNC_SENSING_TASK.period.secs = 10;
  ASYNC_SENSING_TASK.period.nano_secs = 0;
  ASYNC_SENSING_TASK.cpu_reserve.secs = 0;
  ASYNC_SENSING_TASK.cpu_reserve.nano_secs =  500 * NANOS_PER_MS;
  ASYNC_SENSING_TASK.offset.secs = 0;
  ASYNC_SENSING_TASK.offset.nano_secs = 0;
  nrk_activate_task (&ASYNC_SENSING_TASK);

}
