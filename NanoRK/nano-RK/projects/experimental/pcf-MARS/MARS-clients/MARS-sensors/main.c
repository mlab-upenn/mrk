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
#include <nrk_driver_list.h>
#include <nrk_driver.h>
#include <ADXL345.h>
#include <ITG3200.h>
#include <pkt.h>


// if SET_MAC is 0, then read MAC from EEPROM
// otherwise use the coded value
#define SET_MAC  1 
// #define SET_MAC  60 
#define CHAN	25

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

void tdma_error()
{
if(tdma_sync_ok()==0) nrk_led_set(RED_LED);
else nrk_led_clr(RED_LED);
}


int main ()
{
  uint16_t div;
  nrk_setup_ports ();
  nrk_setup_uart (UART_BAUDRATE_115K2);
  begin();//start I2C

  nrk_init ();

  nrk_led_clr (0);
  nrk_led_clr (1);
  nrk_led_clr (2);
  nrk_led_clr (3);

  nrk_time_set (0, 0);

  tdma_set_error_callback(&tdma_error);
  tdma_task_config();

  //nrk_register_drivers();
  nrk_create_taskset ();
  nrk_start ();

  return 0;
}

void my_timer_callback()
{
	nrk_led_toggle(GREEN_LED);
	//nrk_led_toggle(GREEN_LED);
	//nrk_gpio_toggle(NRK_DEBUG_0);
	// Normally you should not call long functions like printf
	// inside a interrupt callback
	//nrk_kprintf( PSTR("*** Timer interrupt!\r\n"));
}
void rx_task ()
{
  nrk_time_t t;
  uint16_t cnt;
  int8_t v,fd;
  uint8_t len, i;
  uint8_t chan;


  cnt = 0;
  nrk_kprintf (PSTR ("Nano-RK Version "));
  printf ("%d\r\n", NRK_VERSION);


  printf ("RX Task PID=%u\r\n", nrk_get_pid ());
  t.secs = 5;
  t.nano_secs = 0;

  chan = CHAN;
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
      tdma_rx_pkt_release();
      /*if((rx_pkt.dst_mac&0xff) == (mac_address&0xff)) 
      {
      printf ("len: %u\r\npayload: ", len);
      for (i = 0; i < len; i++)
        printf ("%d ", rx_buf[i]);
      printf ("\r\n");
      }*/
    }
    //  nrk_wait_until_next_period();
  }
}


void tx_task ()
{
  uint8_t j, i, val, cnt;
  int8_t len;
  int8_t v,fd;
  uint8_t buf[2];
  nrk_sig_mask_t ret;
  nrk_time_t t;
  int8_t* success;
  int16_t acbuf[3];
  int gyrobuf[3];
  int magbuf[3];

	/* Setup application timer with:
	 * Prescaler = 5 
	 * Compare Match = 2500
	 * Sys Clock = 1.0 MHz according to Mega128RFA1 manual pg149
	 * Prescaler 5 means divide sys clock by 1024
	 * 1000000 / 1024 = 976.5625 Hz clock
	 * 1 / 976.5625 = 1.024 ms per tick
	 * 1.024 ms * 1000 = ~1024 ms / per interrupt callback
	 */
	
	val=nrk_timer_int_configure(NRK_APP_TIMER_0, 5, 1000, &my_timer_callback );
	if(val==NRK_OK) nrk_kprintf( PSTR("Callback timer setup\r\n"));
	else nrk_kprintf( PSTR("Error setting up timer callback\r\n"));
	
	// Zero the timer...
	nrk_timer_int_reset(NRK_APP_TIMER_0);
	// Start the timer...
	nrk_timer_int_start(NRK_APP_TIMER_0);

  // Set Port D.0 as input
  // On the FireFly nodes we use this for motion or syntonistor input
  // It can be interrupt driven, but we don't do this with TDMA since updates are so fast anyway
  //  DDRD &= ~(0x1);
  	
   
   //nrk_kprintf( PSTR("Booting Up ADXL345. . .\r\n") );
  /**** Start Accelerometer and check connections 
   * Start up accelerometer
   * Ensure ADXL is connected. 
   * Enable continous measurement
   * INT_SCALER =  10000 
   * Set accelerometer range to maximum og 2G
   * Set sample rate to 1.6Khz
   ****/	
   set_up_ADXL345();
	
  /**** Start Gyro and check connections 
   * Start up gyro
   * Zero calibrate gyro.
   * INT_SCALER_GYRO =  100  
   * Leave gyro still for about 2 seconds to zero-calibrate
   * sets gyro sample rate to 8Khz with a 256Hz BW(bandwidth) LP (low pass) filter
   ****/
   set_up_ITG3200();	
	
  /**** Start magnetometer and check connections 
   * set scale to +/- 1.3 gauss
   * set measurement mode to continous	
   * INT_SCALER_MAG = 1000
   * sets sample rate to highest value of 75Hz
   * sets averaging value to 8 samples per reading givein at 75 Hz
   ****/
   success = set_up_HMC5883L();
   
   if (*success) nrk_kprintf( PSTR("HMC5883L(Magnetometer) boot success. Scale +/- 1.3 Ga. . .\r\n"));
   if ( *(success+1) ) nrk_kprintf( PSTR("HMC5883L measurement mode: continuous . .\r\n") );
	
	//printf( "My node's address is %d\r\n",NODE_ADDR );

  printf ("tx_task PID=%d\r\n", nrk_get_pid ());

   // setup a software watch dog timer
   t.secs=10;
   t.nano_secs=0;
   nrk_sw_wdt_init(0, &t, NULL);
   nrk_sw_wdt_start(0);

   while (!tdma_started ())
   nrk_wait_until_next_period ();

	

  while (1) {

	//nrk_led_toggle(GREEN_LED);


	tx_pkt.payload[0] = 1;  // ELEMENTS
	tx_pkt.payload[1] = 3;  // Key

   /* 	fd=nrk_open(FIREFLY_3_SENSOR_BASIC,READ);
        if(fd==NRK_ERROR) nrk_kprintf(PSTR("Failed to open sensor driver\r\n"));
//	val=nrk_set_status(fd,SENSOR_SELECT,BAT);
//	val=nrk_read(fd,&buf,2);
//	tx_pkt.payload[2]=buf[1];
//	tx_pkt.payload[3]=buf[0];
	//printf( "Task bat=%d",buf);
	tx_pkt.payload[2]=0;
	tx_pkt.payload[3]=0;
	
	//val=nrk_set_status(fd,SENSOR_SELECT,LIGHT);
	//val=nrk_read(fd,&buf,2);
	tx_pkt.payload[4]=buf[1];
	tx_pkt.payload[5]=buf[0];
	//printf( " light=%d",buf);
	
	//val=nrk_set_status(fd,SENSOR_SELECT,TEMP);
	//val=nrk_read(fd,&buf,2);
	tx_pkt.payload[6]=buf[1];
	tx_pkt.payload[7]=buf[0];
	//printf( " temp=%d",buf);
	
	//val=nrk_set_status(fd,SENSOR_SELECT,ACC_X);
	//val=nrk_read(fd,&buf,2);
	tx_pkt.payload[8]=buf[1];
	tx_pkt.payload[9]=buf[0];
	//printf( " acc_x=%d",buf);
	
	//val=nrk_set_status(fd,SENSOR_SELECT,ACC_Y);
	//val=nrk_read(fd,&buf,2);
	tx_pkt.payload[10]=buf[1];
	tx_pkt.payload[11]=buf[0];
	//printf( " acc_y=%d",buf);
	
	//val=nrk_set_status(fd,SENSOR_SELECT,ACC_Z);
	//val=nrk_read(fd,&buf,2);
	tx_pkt.payload[12]=buf[1];
	tx_pkt.payload[13]=buf[0];
	//printf( " acc_z=%d",buf);
	
	/*
	val=nrk_set_status(fd,SENSOR_SELECT,HUMIDITY);
	val=nrk_read(fd,&buf,4);
	tx_pkt.payload[16]=buf[3];
	tx_pkt.payload[17]=buf[2];
	tx_pkt.payload[18]=buf[1];
	tx_pkt.payload[19]=buf[0];


	val=nrk_set_status(fd,SENSOR_SELECT,TEMP2);
	val=nrk_read(fd,&buf,4);
	tx_pkt.payload[20]=buf[3];
	tx_pkt.payload[21]=buf[2];
	tx_pkt.payload[22]=buf[1];
	tx_pkt.payload[23]=buf[0];


	val=nrk_set_status(fd,SENSOR_SELECT,PRESS);
	val=nrk_read(fd,&buf,4);
	tx_pkt.payload[24]=buf[3];
	tx_pkt.payload[25]=buf[2];
	tx_pkt.payload[26]=buf[1];
	tx_pkt.payload[27]=buf[0];

	//val=nrk_set_status(fd,SENSOR_SELECT,MOTION);
	//val=nrk_read(fd,&buf,2);
	tx_pkt.payload[28]=buf[1];
	tx_pkt.payload[29]=buf[0];

	//val=nrk_set_status(fd,SENSOR_SELECT,AUDIO_P2P);
	//val=nrk_read(fd,&buf,2);
	tx_pkt.payload[14]=buf[1];
	tx_pkt.payload[15]=buf[0];


	// GPIO data
	//gpio_pin = !!(PINE & 0x4);
	//tx_pkt.payload[30]=gpio_pin;
	//printf( " audio=%d\r\n",buf);

    	nrk_close(fd);*/
	  tx_pkt.payload[30]=30;
	  tx_pkt.payload[29]=29;
	  tx_pkt.payload[28]=8;
	  tx_pkt.payload[27]=0;
	  tx_pkt.payload[26]=0;
	  tx_pkt.payload[25]=29;
	  tx_pkt.payload[24]=8;
	  tx_pkt.payload[23]=0;
	  tx_pkt.payload[22]=0;
	  tx_pkt.payload[21]=8;
	  tx_pkt.payload[20]=0;
	  tx_pkt.payload[19]=0;
	  tx_pkt.payload[18]=0;
	  tx_pkt.payload[17]=0;
	  tx_pkt.payload[16]=0;
      tx_pkt.payload[15]=60;
	  tx_pkt.payload[14]=0;
	  tx_pkt.payload[13]=0;
	  tx_pkt.payload[12]=0;
	  tx_pkt.payload[11]=0;
	  tx_pkt.payload[10]=0;
	  tx_pkt.payload[9]=0;
	  tx_pkt.payload[8]=0;
	  tx_pkt.payload[7]=155;
	  tx_pkt.payload[6]=45;
	  tx_pkt.payload[5]=8;
	  tx_pkt.payload[4]=5;
	  tx_pkt.payload[3]=3;
	  tx_pkt.payload[2]=2;

  	 tx_pkt.payload_len=31;
	 tx_pkt.src_mac=mac_address;
	 tx_pkt.dst_mac=0;
	 tx_pkt.type=APP;

    len=pkt_to_buf(&tx_pkt,&tx_buf );

 if(len>0){
    v = tdma_send (&tx_tdma_fd, &tx_buf, len, TDMA_BLOCKING);
    if (v == NRK_OK) {
 	 }
 }
 else { 
	nrk_kprintf(PSTR("Pkt tx error\r\n")); 
	nrk_wait_until_next_period(); 
 }
   

	nrk_sw_wdt_update(0);  
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
  RX_TASK.cpu_reserve.secs = 1;
  RX_TASK.cpu_reserve.nano_secs = 0;
  RX_TASK.offset.secs = 0;
  RX_TASK.offset.nano_secs = 0;
  nrk_activate_task (&RX_TASK);

  TX_TASK.task = tx_task;
  nrk_task_set_stk (&TX_TASK, tx_task_stack, NRK_APP_STACKSIZE);
  TX_TASK.prio = 3;
  TX_TASK.FirstActivation = TRUE;
  TX_TASK.Type = BASIC_TASK;
  TX_TASK.SchType = PREEMPTIVE;
  TX_TASK.period.secs = 1;
  TX_TASK.period.nano_secs = 0;
  TX_TASK.cpu_reserve.secs = 3;
  TX_TASK.cpu_reserve.nano_secs = 0;
  TX_TASK.offset.secs = 0;
  TX_TASK.offset.nano_secs = 0;
  nrk_activate_task (&TX_TASK);
}

