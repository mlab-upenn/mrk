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


#include <include.h>
#include <ulib.h>
#include <stdio.h>
#include <hal.h>
#include <basic_rf.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>


uint8_t send_potato(uint16_t dest_addr);
uint8_t get_potato(void);


RF_TX_INFO rfTxInfo;
RF_RX_INFO rfRxInfo;
uint8_t tx_buf[RF_MAX_PAYLOAD_SIZE];
uint8_t rx_buf[RF_MAX_PAYLOAD_SIZE];



int main (void)
{
	uint16_t my_addr, other_addr, pan_id;
	uint8_t i, channel;

  nrk_setup_ports(); 
  nrk_setup_uart (UART_BAUDRATE_115K2);
 
  printf("Starting 'Hot Potato' Game\r\n"); 
  nrk_led_clr(0); 
  nrk_led_clr(1); 
  nrk_led_clr(2); 
  nrk_led_clr(3); 

	my_addr = 0x1210;
	other_addr = my_addr ^ 0x0001;
	pan_id = 0x0FA1;
	channel = 13;

  rfRxInfo.pPayload = rx_buf;
  rfRxInfo.max_length = RF_MAX_PAYLOAD_SIZE;
		
	nrk_int_enable();
  rf_init(&rfRxInfo, channel, pan_id, my_addr);
	rf_rx_on();
   
	/* Player with even MAC address
	 * starts off with the hot potato */
	if((my_addr % 2) == 0){
		nrk_led_set(RED_LED);
		while(send_potato(other_addr) != 1)
			continue;
		printf("POTATO SENT\r\n");
		nrk_led_clr(RED_LED);
	}

	/* After the player catches the potato he
	 * waits a bit and then throws it back */
	while(1){
		while(get_potato() != 1)
			continue;
		printf("POTATO RECEIVED\r\n");
		nrk_led_set(RED_LED);
		
		/* Wait a bit */
		for(i=0; i<40; i++)
			halWait(10000);
		
		while(send_potato(other_addr) != 1)
			continue;
		printf("POTATO SENT\r\n");
		nrk_led_clr(RED_LED);
	}

}


uint8_t send_potato(uint16_t dest_addr)
{
	sprintf(tx_buf, "Hot Potato!");
	rfTxInfo.pPayload = tx_buf;
	rfTxInfo.length = strlen(tx_buf) + 1;
	rfTxInfo.cca = 1;
	rfTxInfo.ackRequest = 1;
	rfTxInfo.destAddr = dest_addr;
		
	if(rf_tx_packet(&rfTxInfo) == 1)
		return 1;
	else
		return -1;
}



uint8_t get_potato(void)
{
	/* Return an error if there is no new packet
	 * or if the new packet does not contain the
	 * potato. Return success otherwise. */
	if(!rf_rx_packet_nonblock())
		return -1;
	else if(strcmp(rfRxInfo.pPayload, "Hot Potato!"))
		return -2;
	else
		return 1;
}




