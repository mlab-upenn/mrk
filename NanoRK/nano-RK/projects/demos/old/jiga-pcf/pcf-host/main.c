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


NRK_STK rx_task_stack[NRK_APP_STACKSIZE];
nrk_task_type rx_task_info;
void rx_task (void);

NRK_STK tx_task_stack[NRK_APP_STACKSIZE];
nrk_task_type tx_task_info;
void tx_task (void);

void nrk_create_taskset ();


tdma_info tx_tdma_fd;
tdma_info rx_tdma_fd;

uint8_t rx_buf[TDMA_MAX_PKT_SIZE];
uint8_t tx_buf[TDMA_MAX_PKT_SIZE];

int main ()
{
  nrk_setup_ports ();
  nrk_setup_uart (UART_BAUDRATE_115K2);

  nrk_init ();

  nrk_led_clr (ORANGE_LED);
  nrk_led_clr (BLUE_LED);
  nrk_led_clr (GREEN_LED);
  nrk_led_clr (RED_LED);

  nrk_time_set (0, 0);
  nrk_create_taskset ();
  nrk_start ();

  return 0;
}

void tx_task ()
{
  int8_t v,state,outlet_state,dst_mac, outlet;
  uint8_t len, cnt;
  nrk_sig_t uart_rx_signal;
  char c;

  printf ("Gateway Tx Task PID=%u\r\n", nrk_get_pid ());

  while (!tdma_started ())
    nrk_wait_until_next_period ();

  uart_rx_signal=nrk_uart_rx_signal_get();
  nrk_signal_register(uart_rx_signal);

  cnt = 0;
  state=0;

  while (1) {

    if(nrk_uart_data_ready(NRK_DEFAULT_UART))
    {
	c=getchar();
	if(state==1) { 
		dst_mac=c;
		state=2;
	} else if(state==2) {
		outlet=c;
		state=3;
	} else if(state==3)
	{
		outlet_state=c;
		state=4;
	}
	if(c=='S') state=1;
	if(c=='E') {
		if(state==4)
		{
			printf( "TX: %d %d %d\r\n",dst_mac, outlet, outlet_state );
    			tx_buf[0]=dst_mac;
    			tx_buf[1]=outlet;
    			tx_buf[2]=outlet_state;
    			len=3;
    			// Only transmit data if you want to do so
    			// Messages from the host are always broadcasts
    			v = tdma_send (&tx_tdma_fd, &tx_buf, len, TDMA_BLOCKING);
    			if (v == NRK_OK) {
      				nrk_kprintf (PSTR ("Host Packet Sent\n"));
    			}
		}
		state=0;

	}


    } else nrk_event_wait(SIG(uart_rx_signal));

  }
}

void rx_task ()
{
  nrk_time_t t;
  uint16_t cnt;
  int8_t v;
  uint8_t len, i;


  cnt = 0;
  nrk_kprintf (PSTR ("Nano-RK Version "));
  printf ("%d\r\n", NRK_VERSION);


  printf ("RX Task PID=%u\r\n", nrk_get_pid ());
  t.secs = 5;
  t.nano_secs = 0;

// setup a software watch dog timer
//nrk_sw_wdt_init(0, &t, NULL);
// nrk_sw_wdt_start(0);

  tdma_init (TDMA_HOST, 13, 0);

// Change these parameters at runtime...
  tdma_set_slot_len_ms (10);
  tdma_set_slots_per_cycle (32);

  while (!tdma_started ())
    nrk_wait_until_next_period ();


  while (1) {
    // Update watchdog timer
    // nrk_sw_wdt_update(0);
    v = tdma_recv (&rx_tdma_fd, &rx_buf, &len, TDMA_BLOCKING);
    if (v == NRK_OK) {
    //  nrk_kprintf (PSTR ("Got pkt "));
    //  printf ("src: %u rssi: %d ", rx_tdma_fd.src, rx_tdma_fd.rssi);
    //  printf ("slot: %u ", rx_tdma_fd.slot);
    //  printf ("len: %u\r\nS ", len);
      printf ("S " );
      for (i = 0; i < len; i++)
        printf ("%d ", rx_buf[i]);
      printf ("\r\n");
    }

    //  nrk_wait_until_next_period();
  }
}

void nrk_create_taskset ()
{
  nrk_task_set_entry_function (&rx_task_info, rx_task);
  nrk_task_set_stk (&rx_task_info, rx_task_stack, NRK_APP_STACKSIZE);
  rx_task_info.prio = 1;
  rx_task_info.FirstActivation = TRUE;
  rx_task_info.Type = BASIC_TASK;
  rx_task_info.SchType = PREEMPTIVE;
  rx_task_info.period.secs = 0;
  rx_task_info.period.nano_secs = 250 * NANOS_PER_MS;
  rx_task_info.cpu_reserve.secs = 0;
  rx_task_info.cpu_reserve.nano_secs = 100 * NANOS_PER_MS;
  rx_task_info.offset.secs = 0;
  rx_task_info.offset.nano_secs = 0;
  nrk_activate_task (&rx_task_info);

  nrk_task_set_entry_function (&tx_task_info, tx_task);
  nrk_task_set_stk (&tx_task_info, tx_task_stack, NRK_APP_STACKSIZE);
  tx_task_info.prio = 1;
  tx_task_info.FirstActivation = TRUE;
  tx_task_info.Type = BASIC_TASK;
  tx_task_info.SchType = PREEMPTIVE;
  tx_task_info.period.secs = 0;
  tx_task_info.period.nano_secs = 250 * NANOS_PER_MS;
  tx_task_info.cpu_reserve.secs = 0;
  tx_task_info.cpu_reserve.nano_secs = 100 * NANOS_PER_MS;
  tx_task_info.offset.secs = 0;
  tx_task_info.offset.nano_secs = 0;
  nrk_activate_task (&tx_task_info);

  tdma_task_config ();

}
