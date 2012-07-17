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
#include <pcf_tdma.h>
#include <nrk_error.h>
#include <nrk_timer.h>
#include <nrk_eeprom.h>
#include <pkt.h>
#include <nrk_driver_list.h>
#include <nrk_driver.h>
#include <ff_basic_sensor.h>


// if SET_MAC is 0, then read MAC from EEPROM
// otherwise use the coded value
 #define SET_MAC  0x0000

PKT_T	tx_pkt;
PKT_T	rx_pkt;
uint8_t gpio_pin;

tdma_info tx_tdma_fd;
tdma_info rx_tdma_fd;

uint8_t rx_buf[TDMA_MAX_PKT_SIZE];
uint8_t tx_buf[TDMA_MAX_PKT_SIZE];

uint32_t mac_address;


nrk_task_type RX_TASK;
NRK_STK rx_task_stack[NRK_APP_STACKSIZE];
void rx_task (void);


nrk_task_type TX_TASK;
NRK_STK tx_task_stack[NRK_APP_STACKSIZE];
void tx_task (void);

void nrk_create_taskset ();
void nrk_register_drivers();


int main ()
{
  uint16_t div;
  nrk_setup_ports ();
  nrk_setup_uart (UART_BAUDRATE_115K2);

  nrk_init ();

  nrk_led_clr (0);
  nrk_led_clr (1);
  nrk_led_clr (2);
  nrk_led_clr (3);

  nrk_time_set (0, 0);

  tdma_task_config();

  nrk_register_drivers();
  nrk_create_taskset ();
  nrk_start ();

  return 0;
}

void rx_task ()
{
  nrk_time_t t;
  uint16_t cnt;
  int8_t v;
  uint8_t len, i;
uint8_t chan;


  cnt = 0;
  nrk_kprintf (PSTR ("Nano-RK Version "));
  printf ("%d\r\n", NRK_VERSION);


  printf ("RX Task PID=%u\r\n", nrk_get_pid ());
  t.secs = 5;
  t.nano_secs = 0;

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



  tdma_init (TDMA_CLIENT, chan, mac_address);


  while (!tdma_started ())
    nrk_wait_until_next_period ();

  // Set TDMA slot to lower byte of MAC address
  v = tdma_tx_slot_add (mac_address & 0xff);

  if (v != NRK_OK)
    nrk_kprintf (PSTR ("Could not add slot!\r\n"));

  while (1) {
    // Update watchdog timer
    // nrk_sw_wdt_update(0);
    v = tdma_recv (&rx_tdma_fd, &rx_buf, &len, TDMA_BLOCKING);
    if (v == NRK_OK) {
     // printf ("src: %u\r\nrssi: %d\r\n", rx_tdma_fd.src, rx_tdma_fd.rssi);
     // printf ("slot: %u\r\n", rx_tdma_fd.slot);
     // printf ("cycle len: %u\r\n", rx_tdma_fd.cycle_size);
      v=buf_to_pkt(&rx_buf, &rx_pkt);
      if((rx_pkt.dst_mac&0xff) == (mac_address&0xff)) 
      {
      printf ("len: %u\r\npayload: ", len);
      for (i = 0; i < len; i++)
        printf ("%d ", rx_buf[i]);
      printf ("\r\n");




      }


    }

    //  nrk_wait_until_next_period();
  }

}

uint8_t ctr_cnt[4];

void tx_task ()
{
  uint8_t j, i, val, cnt;
  int8_t len;
  int8_t v,fd;
  uint8_t buf[2];
  nrk_sig_t tx_done_signal;
  nrk_sig_mask_t ret;
  nrk_time_t r_period;



  // Set Port D.0 as input
  // On the FireFly nodes we use this for motion or syntonistor input
  // It can be interrupt driven, but we don't do this with TDMA since updates are so fast anyway
  DDRD &= ~(0x1);



  printf ("tx_task PID=%d\r\n", nrk_get_pid ());

  // Wait until the tx_task starts up bmac
  // This should be called by all tasks using bmac that

  while (!tdma_started ())
    nrk_wait_until_next_period ();

 


  while (1) {

    	fd=nrk_open(FIREFLY_SENSOR_BASIC,READ);
    	if(fd==NRK_ERROR) nrk_kprintf(PSTR("Failed to open sensor driver\r\n"));


    	tx_pkt.payload[0] = 1;  // ELEMENTS
   	 tx_pkt.payload[1] = 3;  // Key

	val=nrk_set_status(fd,SENSOR_SELECT,BAT);
	val=nrk_read(fd,&buf,2);
	tx_pkt.payload[2]=buf[1];
	tx_pkt.payload[3]=buf[0];
	//printf( "Task bat=%d",buf);
	val=nrk_set_status(fd,SENSOR_SELECT,LIGHT);
	val=nrk_read(fd,&buf,2);
	tx_pkt.payload[4]=buf[1];
	tx_pkt.payload[5]=buf[0];
	//printf( " light=%d",buf);
	val=nrk_set_status(fd,SENSOR_SELECT,TEMP);
	val=nrk_read(fd,&buf,2);
	tx_pkt.payload[6]=buf[1];
	tx_pkt.payload[7]=buf[0];
	//printf( " temp=%d",buf);
	val=nrk_set_status(fd,SENSOR_SELECT,ACC_X);
	val=nrk_read(fd,&buf,2);
	tx_pkt.payload[8]=buf[1];
	tx_pkt.payload[9]=buf[0];
	//printf( " acc_x=%d",buf);
	val=nrk_set_status(fd,SENSOR_SELECT,ACC_Y);
	val=nrk_read(fd,&buf,2);
	tx_pkt.payload[10]=buf[1];
	tx_pkt.payload[11]=buf[0];
	//printf( " acc_y=%d",buf);
	val=nrk_set_status(fd,SENSOR_SELECT,ACC_Z);
	val=nrk_read(fd,&buf,2);
	tx_pkt.payload[12]=buf[1];
	tx_pkt.payload[13]=buf[0];
	//printf( " acc_z=%d",buf);
	val=nrk_set_status(fd,SENSOR_SELECT,AUDIO_P2P);
	nrk_spin_wait_us(60000);
	val=nrk_read(fd,&buf,2);
	tx_pkt.payload[14]=buf[1];
	tx_pkt.payload[15]=buf[0];
	// GPIO data
	gpio_pin = (PIND & 0x1);
	tx_pkt.payload[16]=gpio_pin;
	//printf( " audio=%d\r\n",buf);
    	tx_pkt.payload_len=17;

    	nrk_close(fd);

    	tx_pkt.src_mac=mac_address;
    	tx_pkt.dst_mac=0;
    	tx_pkt.type=APP;

    len=pkt_to_buf(&tx_pkt,&tx_buf );
    if(len>0)
    {
    v = tdma_send (&tx_tdma_fd, &tx_buf, len, TDMA_BLOCKING);
    if (v == NRK_OK) {
      //nrk_kprintf (PSTR ("App Packet Sent\n"));
    }
    } else nrk_wait_until_next_period();

  }



}

void nrk_create_taskset ()
{


  RX_TASK.task = rx_task;
  nrk_task_set_stk (&RX_TASK, rx_task_stack, NRK_APP_STACKSIZE);
  RX_TASK.prio = 2;
  RX_TASK.FirstActivation = TRUE;
  RX_TASK.Type = BASIC_TASK;
  RX_TASK.SchType = PREEMPTIVE;
  RX_TASK.period.secs = 1;
  RX_TASK.period.nano_secs = 0;
  RX_TASK.cpu_reserve.secs = 0;
  RX_TASK.cpu_reserve.nano_secs = 0;
  RX_TASK.offset.secs = 0;
  RX_TASK.offset.nano_secs = 0;
  nrk_activate_task (&RX_TASK);

  TX_TASK.task = tx_task;
  nrk_task_set_stk (&TX_TASK, tx_task_stack, NRK_APP_STACKSIZE);
  TX_TASK.prio = 2;
  TX_TASK.FirstActivation = TRUE;
  TX_TASK.Type = BASIC_TASK;
  TX_TASK.SchType = PREEMPTIVE;
  TX_TASK.period.secs = 1;
  TX_TASK.period.nano_secs = 0;
  TX_TASK.cpu_reserve.secs = 0;
  TX_TASK.cpu_reserve.nano_secs = 0;
  TX_TASK.offset.secs = 0;
  TX_TASK.offset.nano_secs = 0;
  nrk_activate_task (&TX_TASK);

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
