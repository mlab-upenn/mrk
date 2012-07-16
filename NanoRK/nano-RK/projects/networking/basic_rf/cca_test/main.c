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


void tx_spam(uint16_t dest_addr);
void rx_print(void);
void cca_loop(void);
void cca_stat(void);
void tx_send_one_packet(void);

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
 
  printf("Starting CCA test...\r\n"); 
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

	/* Depending on their address, one node will
	 * jam the channel and the other one will constanly
	 * perform CCA on it */
	if(my_addr % 2 == 0){
		/*for(i=0; i<16; i++){
			printf("---------------------------\r\n");
			printf("THREE ED TESTS AT LEVEL: %u\r\n", i);
			printf("---------------------------\r\n");
			CCA_THRES = (CCA_THRES & 0xF0) | i;
			cca_stat();
			rx_print();
		}*/
		CCA_THRES = (CCA_THRES & 0xF0) | 5;
		while(1)
			cca_stat();
	}
	else
		tx_spam(other_addr);

	printf("EXECUTION COMPLETE\r\n");
	while(1)
		continue;
}


void tx_spam(uint16_t dest_addr)
{
	rfTxInfo.pPayload = tx_buf;
	rfTxInfo.cca = 0;
	rfTxInfo.ackRequest = 0;
	rfTxInfo.destAddr = dest_addr;
		
	sprintf(tx_buf, "spammmmmmmmm");
	rfTxInfo.length = strlen(tx_buf) + 1;
	while(1){
		rf_tx_packet(&rfTxInfo);
	}
}


void rx_print(void)
{
	uint8_t i;
	
	if(!rf_rx_packet_nonblock())
		return;
	printf("Payload: ");
	for(i=0; i<rfRxInfo.length; i++)
		printf("%c",rfRxInfo.pPayload[i]);
	printf("\r\n\r\n");
}


void cca_loop(void)
{
	uint8_t i;
	uint8_t checks[3];

	while(1){
		/* Send a spam packet */
		tx_send_one_packet();		

   rf_rx_off();
		for(i=0; i<10; i++)
			halWait(10000);
		rf_rx_on();
	
		/* Perform 3 CCA checks */
		for(i=0; i<3; i++){
			checks[i] = rf_cca_check();
			halWait(100);
		}
		for(i=0; i<3; i++){
			printf("rf_cca_check: %d\r\n", checks[i]);
		}
		
		/* Power cycle rx */
		rf_rx_off();
		for(i=0; i<40; i++)
			halWait(10000);
		//rf_rx_on();
	}
}


void cca_stat(void)
{
	uint16_t i, cca_busy_total, cca_busy_length, cca_busy_hist[50], 
					 cca_clear_length, cca_clear_hist[50];
	uint8_t cca_busy;

	cca_busy = 0;
	cca_busy_total = 0;
	cca_busy_length = 0;
	cca_clear_length = 0;
	for(i=0; i<50; i++){
		cca_busy_hist[i] = 0;
		cca_clear_hist[i] = 0;
	}

	printf("CCA Test | ");
	for(i=0; i<50; i++)
		printf(" ");
	printf(" | BUSY\rCCA Test | ");

	/* Perform 3 CCA checks */
	for(i=0; i<65000; i++){
		cca_busy = !rf_cca_check();

		if(cca_busy){
			//if(cca_busy_length > 10 && cca_busy_length < 13)
				nrk_led_set(ORANGE_LED);

			cca_busy_length += 1;
			cca_busy_total += 1;
				
			if(cca_clear_length){
				if(cca_clear_length < 49)
					cca_clear_hist[cca_clear_length - 1] += 1;
				else
					cca_clear_hist[49] += 1;
				cca_clear_length = 0;
			}
		}
		else{
			nrk_led_clr(ORANGE_LED);

			cca_clear_length += 1;
				
			if(cca_busy_length){
				if(cca_busy_length < 49)
					cca_busy_hist[cca_busy_length - 1] += 1;
				else
					cca_busy_hist[49] += 1;
				cca_busy_length = 0;
			}
		}

		if((i % (65000/50)) == 0)
			printf("#");

	}

	nrk_kprintf(PSTR(" | DONE\r\n"));
	
	printf("\r\nStats:\r\n");
	printf("cca busy num: %u\r\n", cca_busy_total);
	printf("cca busy percent: %u", cca_busy_total / (65000/100));
	printf("\r\n\r\n");
	printf("HISTOGRAM BUSY LENGTH:\t\t\tHISTOGRAM CLEAR LENGTH:\r\n\r\n");
		
	for(i=0; i<24; i++){
		printf("[%u]: %u  \t", i+1, cca_busy_hist[i]);
		printf("[%u]: %u  \t|\t", i+1+25, cca_busy_hist[i+25]);
		printf("[%u]: %u  \t", i+1, cca_clear_hist[i]);
		printf("[%u]: %u\r\n", i+1+25, cca_clear_hist[i+25]);
	}
	printf("[%u]: %u  \t", 25, cca_busy_hist[24]);
	printf("[other]: %u\t|\t", cca_busy_hist[49]);
	printf("[%u]: %u  \t", 25, cca_clear_hist[24]);
	printf("[other]: %u\r\n", cca_clear_hist[49]);

	printf("\r\n\r\n\r\n");

	return;

}



void tx_send_one_packet(void)
{
	sprintf(tx_buf, "one spam packet");
	rfTxInfo.pPayload = tx_buf;
	rfTxInfo.length = strlen(tx_buf) + 1;
	rfTxInfo.cca = 0;
	rfTxInfo.ackRequest = 0;
	rfTxInfo.destAddr = 0xBAD;
		
	rf_tx_packet(&rfTxInfo);
}


