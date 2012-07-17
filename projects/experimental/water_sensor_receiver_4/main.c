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
#include <nrk_driver_list.h>
#include <nrk_driver.h>
#include <adc_driver.h>

#include <basic_rf.h>


RF_TX_INFO rfTxInfo;
RF_RX_INFO rfRxInfo;
uint8_t tx_buf[RF_MAX_PAYLOAD_SIZE];
uint8_t rx_buf[RF_MAX_PAYLOAD_SIZE];


int
main ()
{
	uint8_t  i, ccount;
	char c = 0;

  nrk_setup_ports();
	nrk_setup_uart(UART_BAUDRATE_115K2);

  printf( "Starting up...\r\n" );

	nrk_int_enable();

	rfRxInfo.pPayload = rx_buf;
	rfRxInfo.max_length = RF_MAX_PAYLOAD_SIZE;
	rf_init (&rfRxInfo, 13, 0x2420, 0x1215);

	rfTxInfo.pPayload=tx_buf;
	rfTxInfo.destAddr = 0x1214;
	rfTxInfo.cca = 0;
	rfTxInfo.ackRequest = 0;

	nrk_led_set(1);
	
	nrk_led_clr(0);
	nrk_led_clr(2);
	nrk_led_clr(3);

	rf_rx_on();

	ccount = 0;

	while(1){
		if(rf_polling_rx_packet() == 1){

			if(!strncmp("sp,id", rfRxInfo.pPayload, strlen("sp,id"))){
				sprintf((char *) tx_buf, "%s", rfRxInfo.pPayload);
				tx_buf[0] = 'm';
				tx_buf[9] = ';';
				tx_buf[10] = '\0';
				rfTxInfo.length= strlen((char *) tx_buf) + 1;
				rf_tx_packet(&rfTxInfo);
				nrk_led_toggle(0);
			}

			for(i=0; i<rfRxInfo.length; i++ )
				printf( "%c", rfRxInfo.pPayload[i]);
			printf( "\r\n" );
		}

		/* Multiple characters on serial port? */
		if(nrk_uart_data_ready(0) != 0){
			c = getchar();
			if(c == '\n'){
				rfTxInfo.length= strlen((char *) tx_buf) + 1;
				rf_tx_packet(&rfTxInfo);
				
				ccount = 0;
				tx_buf[0] = '\0';
			}
			else if((c != '\r') && (ccount < 100)){
				sprintf(((char *) tx_buf) + ccount,"%c",c);
				ccount++;
			}
		}

	}
  
  return 0;
}


