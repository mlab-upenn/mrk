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

#include <nrk_driver_list.h>
#include <nrk_driver.h>
#include <adc_driver.h>

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
void nrk_register_drivers();

tdma_info tx_tdma_fd;
tdma_info rx_tdma_fd;

uint8_t rx_buf[TDMA_MAX_PKT_SIZE];
uint8_t tx_buf[TDMA_MAX_PKT_SIZE];

uint16_t adc_buf[8];

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
  nrk_register_drivers();
  nrk_create_taskset ();
  nrk_start();
  
  return 0;
}

void tx_task()
{
int8_t v,fd;
uint8_t len,cnt,chan;

  
printf( "Tx Task PID=%u\r\n",nrk_get_pid());

while(!tdma_started()) nrk_wait_until_next_period();

v=tdma_tx_slot_add(mac_address);

  // Open ADC device as read 
  fd=nrk_open(ADC_DEV_MANAGER,READ);
  if(fd==NRK_ERROR) nrk_kprintf( PSTR("Failed to open ADC driver\r\n"));

  cnt=0;
  chan=0;

  while(1) {
	
	for(chan=0; chan<8; chan++ )
	{
	  // Example of setting the ADC channel
	  v=nrk_set_status(fd,ADC_CHAN,chan);
	  if(v==NRK_ERROR) nrk_kprintf( PSTR("Failed to set ADC status\r\n" ));
	  v=nrk_read(fd,&adc_buf[chan],2);
	  if(v==NRK_ERROR) nrk_kprintf( PSTR("Failed to read ADC\r\n" ));
	}
 	
	for(chan=0; chan<8; chan++ )
	{
		tx_buf[chan*2]=(adc_buf[chan]>>8) & 0xff;
		tx_buf[chan*2+1]=((uint8_t)adc_buf[chan]) & 0xff;
		printf( "chan%d: %d ",chan,adc_buf[chan]);
	}
	nrk_kprintf( PSTR("\r\n" ));
	len=16;

	 v=tdma_send(&tx_tdma_fd, &tx_buf, len, TDMA_BLOCKING );	
	if(v==NRK_OK)
		{
		nrk_kprintf( PSTR("App Packet Sent\r\n"));
		}
	}
}

void rx_task()
{
nrk_time_t t;
uint16_t cnt;
int8_t v;
uint8_t len,i;
uint8_t chan;


cnt=0;
nrk_kprintf( PSTR("Nano-RK Version ") );
printf( "%d\r\n",NRK_VERSION );

  
printf( "RX Task PID=%u\r\n",nrk_get_pid());
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


tdma_init(TDMA_CLIENT, chan, mac_address);


while(!tdma_started()) nrk_wait_until_next_period();


if(v!=NRK_OK ) nrk_kprintf(PSTR("Could not add slot!\r\n"));

  while(1) {
	// Update watchdog timer
	// nrk_sw_wdt_update(0);
	v=tdma_recv(&rx_tdma_fd, &rx_buf, &len, TDMA_BLOCKING );	
	if(v==NRK_OK)
	{
		printf( "src: %u\r\nrssi: %d\r\n",rx_tdma_fd.src, rx_tdma_fd.rssi );
		printf( "slot: %u\r\n",rx_tdma_fd.slot);
		printf( "cycle len: %u\r\n",rx_tdma_fd.cycle_size);
		printf( "len: %u\r\npayload: ",len);
		for(i=0; i<len; i++ )
			printf("%c",rx_buf[i] );
		printf( "\r\n" );
	}

	//  nrk_wait_until_next_period();
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
  rx_task_info.period.secs = 0;
  rx_task_info.period.nano_secs = 250*NANOS_PER_MS;
  rx_task_info.cpu_reserve.secs = 1;
  rx_task_info.cpu_reserve.nano_secs = 50*NANOS_PER_MS;
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

void nrk_register_drivers()
{
int8_t val;

// Register the ADC device driver
// Make sure to add: 
//     #define NRK_MAX_DRIVER_CNT  
//     in nrk_cfg.h
// Make sure to add: 
//     SRC += $(ROOT_DIR)/src/drivers/platform/$(PLATFORM_TYPE)/source/adc_driver.c
//     in makefile
val=nrk_register_driver( &dev_manager_adc,ADC_DEV_MANAGER);
if(val==NRK_ERROR) nrk_kprintf( PSTR("Failed to load my ADC driver\r\n") );

}


