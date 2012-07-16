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
#include <hal.h>
#include <nrk_error.h>
#include <nrk_timer.h>
#include <nrk_stack_check.h>
#include <nrk_stats.h>
#include <pcf_tdma.h>
#include <nrk_eeprom.h>

// if SET_MAC is 0, then read MAC from EEPROM
// otherwise use the coded value
#define SET_MAC  0x0000

uint32_t mac_address;

NRK_STK rx_task_stack[NRK_APP_STACKSIZE];
nrk_task_type rx_task_info;
void rx_task(void);

NRK_STK tx_task_stack[NRK_APP_STACKSIZE];
nrk_task_type tx_task_info;
void tx_task(void);

void nrk_create_taskset();


tdma_info tx_tdma_fd;
tdma_info rx_tdma_fd;

uint8_t rx_buf[TDMA_MAX_PKT_SIZE];
uint8_t tx_buf[TDMA_MAX_PKT_SIZE];

int
main ()
{
  nrk_setup_ports();
  nrk_setup_uart(UART_BAUDRATE_115K2);

  nrk_init();

  nrk_led_clr(ORANGE_LED);
  nrk_led_clr(BLUE_LED);
  nrk_led_clr(GREEN_LED);
  nrk_led_clr(RED_LED);
 
  nrk_time_set(0,0);
  nrk_create_taskset ();
  nrk_start();
  
  return 0;
}

void tx_task()
{
int8_t v;
uint8_t len,cnt;

  
printf( "Gateway Tx Task PID=%u\r\n",nrk_get_pid());

while(!tdma_started()) nrk_wait_until_next_period();

cnt=0;

  while(1) {
	// This is simply a place holder in case you want to add Host -> Client Communication
	  /*	
	sprintf( tx_buf,"This is a test %d\n",cnt );
	cnt++;
	len=strlen(tx_buf)+1;

	 v=tdma_send(&tx_tdma_fd, &tx_buf, len, TDMA_BLOCKING );	
	if(v==NRK_OK)
		{
		nrk_kprintf( PSTR("App Packet Sent\n"));
		}
	*/
	nrk_led_toggle(BLUE_LED);
	nrk_wait_until_next_period();
	}
}

void rx_task()
{
nrk_time_t t;
uint16_t cnt;
int8_t v;
uint8_t len,i,chan;


cnt=0;
nrk_kprintf( PSTR("Nano-RK Version ") );
printf( "%d\r\n",NRK_VERSION );

  
printf( "Gateway Task PID=%u\r\n",nrk_get_pid());
t.secs=5;
t.nano_secs=0;

// setup a software watch dog timer
//nrk_sw_wdt_init(0, &t, NULL);
// nrk_sw_wdt_start(0);

  chan = 25;
  if (SET_MAC == 0x00) {

    v = read_eeprom_mac_address (&mac_address);
    if (v == NRK_OK) {
      v = read_eeprom_channel (&chan);
    }
    else {
      while (1) {
        nrk_kprintf (PSTR
                     ("* ERROR reading MAC address, run eeprom-set utility\r\n"));
        nrk_wait_until_next_period ();
      }
    }
  }
  else
    mac_address = SET_MAC;

  printf ("MAC ADDR: %x\r\n", mac_address & 0xffff);
  printf ("chan = %d\r\n", chan);


tdma_init(TDMA_HOST, chan, mac_address);

// Change these parameters anytime you want...
tdma_set_slot_len_ms(10);
tdma_set_slots_per_cycle(100);

while(!tdma_started()) nrk_wait_until_next_period();


  while(1) {
	v=tdma_recv(&rx_tdma_fd, &rx_buf, &len, TDMA_BLOCKING );	
	nrk_led_set(GREEN_LED);
	if(v==NRK_OK)
	{
		for(i=0; i<len; i++ ) printf( "%c", rx_buf[i]);
	}
	else tdma_rx_pkt_release();

	  nrk_led_clr(GREEN_LED);
	}
}

void
nrk_create_taskset()
{
  nrk_task_set_entry_function( &rx_task_info, rx_task);
  nrk_task_set_stk( &rx_task_info, rx_task_stack, NRK_APP_STACKSIZE);
  rx_task_info.prio = 1;
  rx_task_info.FirstActivation = TRUE;
  rx_task_info.Type = BASIC_TASK;
  rx_task_info.SchType = PREEMPTIVE;
  rx_task_info.period.secs = 1;
  rx_task_info.period.nano_secs = 0;
  rx_task_info.cpu_reserve.secs = 0;
  rx_task_info.cpu_reserve.nano_secs = 500*NANOS_PER_MS;
  rx_task_info.offset.secs = 0;
  rx_task_info.offset.nano_secs= 0;
  nrk_activate_task (&rx_task_info);

  nrk_task_set_entry_function( &tx_task_info, tx_task);
  nrk_task_set_stk( &tx_task_info, tx_task_stack, NRK_APP_STACKSIZE);
  tx_task_info.prio = 1;
  tx_task_info.FirstActivation = TRUE;
  tx_task_info.Type = BASIC_TASK;
  tx_task_info.SchType = PREEMPTIVE;
  tx_task_info.period.secs = 0;
  tx_task_info.period.nano_secs = 250*NANOS_PER_MS;
  tx_task_info.cpu_reserve.secs = 1;
  tx_task_info.cpu_reserve.nano_secs = 50*NANOS_PER_MS;
  tx_task_info.offset.secs = 0;
  tx_task_info.offset.nano_secs= 0;
  nrk_activate_task (&tx_task_info);

  tdma_task_config();

}


