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
#include <power_driver.h>
#include <nrk_eeprom.h>

// if SET_MAC is 0, then read MAC from EEPROM
// otherwise use the coded value
 #define SET_MAC  0x0000


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



  tdma_init (TDMA_CLIENT, 13, mac_address);


  while (!tdma_started ())
    nrk_wait_until_next_period ();

  v = tdma_tx_slot_add (mac_address);

  if (v != NRK_OK)
    nrk_kprintf (PSTR ("Could not add slot!\r\n"));

  while (1) {
    // Update watchdog timer
    // nrk_sw_wdt_update(0);
    v = tdma_recv (&rx_tdma_fd, &rx_buf, &len, TDMA_BLOCKING);
    if (v == NRK_OK) {
      printf ("src: %u\r\nrssi: %d\r\n", rx_tdma_fd.src, rx_tdma_fd.rssi);
      printf ("slot: %u\r\n", rx_tdma_fd.slot);
      printf ("cycle len: %u\r\n", rx_tdma_fd.cycle_size);
      printf ("len: %u\r\npayload: ", len);
      for (i = 0; i < len; i++)
        printf ("%d ", rx_buf[i]);
      printf ("\r\n");

      if(rx_buf[0]==(mac_address&0xff))
      {
	if(rx_buf[2]==0) {
		power_socket_disable(rx_buf[1]);	
		printf( "Disable %d\r\n", rx_buf[1] );
	}
	if(rx_buf[2]==1) {
		power_socket_enable(rx_buf[1]);	
		printf( "Enable %d\r\n", rx_buf[1] );
	}
      }
    }

    //  nrk_wait_until_next_period();
  }

}

uint8_t ctr_cnt[4];

void tx_task ()
{
  uint8_t j, i, val, len, cnt;
  int8_t v;
  nrk_sig_t tx_done_signal;
  nrk_sig_mask_t ret;
  nrk_time_t r_period;

  //power_socket_disable(0);
  //power_socket_disable(1);
  printf ("tx_task PID=%d\r\n", nrk_get_pid ());

  // Wait until the tx_task starts up bmac
  // This should be called by all tasks using bmac that

  while (!tdma_started ())
    nrk_wait_until_next_period ();


  power_init ();

  /*
  for (i = 0; i < 3; i++) {
    power_socket_enable (0);
    power_socket_enable (1);
    nrk_wait_until_next_period ();
    power_socket_disable (0);
    power_socket_disable (1);
    nrk_wait_until_next_period ();
  }
*/
  power_socket_enable (0);
  power_socket_enable (1);

  //nrk_kprintf( PSTR("after outlet on\r\n"));

  // Sample of using Reservations on TX packets
  // This example allows 2 packets to be sent every 5 seconds
  // r_period.secs=5;
  // r_period.nano_secs=0;
  // v=bmac_tx_reserve_set( &r_period, 2 );
  // if(v==NRK_ERROR) nrk_kprintf( PSTR("Error setting b-mac tx reservation (is NRK_MAX_RESERVES defined?)\r\n" ));



  while (1) {


    // For blocking transmits, use the following function call.
    // For this there is no need to register  
    tx_buf[0] = (rms_current >> 8) & 0xff;
    tx_buf[1] = rms_current & 0xff;
    tx_buf[2] = (rms_voltage >> 8) & 0xff;
    tx_buf[3] = rms_voltage & 0xff;
    tx_buf[4] = freq & 0xff;
    tx_buf[5] = (true_power >> 16) & 0xff;
    tx_buf[6] = (true_power >> 8) & 0xff;
    tx_buf[7] = (true_power) & 0xff;
    tx_buf[8] = (tmp_energy >> 24) & 0xff;
    tx_buf[9] = (tmp_energy >> 16) & 0xff;
    tx_buf[10] = (tmp_energy >> 8) & 0xff;
    tx_buf[11] = (tmp_energy) & 0xff;
    tx_buf[12] = (l_v_p2p_high >> 8) & 0xff;
    tx_buf[13] = (l_v_p2p_high) & 0xff;
    tx_buf[14] = (l_v_p2p_low >> 8) & 0xff;
    tx_buf[15] = (l_v_p2p_low) & 0xff;
    tx_buf[16] = (l_c_p2p_high >> 8) & 0xff;
    tx_buf[17] = (l_c_p2p_high) & 0xff;
    tx_buf[18] = (l_c_p2p_low >> 8) & 0xff;
    tx_buf[19] = (l_c_p2p_low) & 0xff;
    tx_buf[20] = (total_secs >> 24) & 0xff;
    tx_buf[21] = (total_secs >> 16) & 0xff;
    tx_buf[22] = (total_secs >> 8) & 0xff;
    tx_buf[23] = (total_secs) & 0xff;
    tx_buf[24] = (mac_address & 0xff);
    tx_buf[25] = socket_0_active;
    tx_buf[26] = socket_1_active;

    v = tdma_send (&tx_tdma_fd, &tx_buf, 27, TDMA_BLOCKING);
    if (v == NRK_OK) {
      nrk_kprintf (PSTR ("App Packet Sent\n"));
    }

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
