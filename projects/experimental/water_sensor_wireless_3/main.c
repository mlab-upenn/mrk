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


typedef struct{
	uint16_t cond;
	uint16_t temp;
} data_point_t;

#define BANKSIZE 2048

data_point_t databank[BANKSIZE];
uint16_t numpoints;
nrk_sem_t *comm_sem;



NRK_STK Stack1[NRK_APP_STACKSIZE];
nrk_task_type TaskOne;
void Task1(void);


NRK_STK Stack2[NRK_APP_STACKSIZE];
nrk_task_type TaskTwo;
void Task2(void);


void nrk_create_taskset();
void nrk_register_drivers();
uint8_t kill_stack(uint8_t val);


RF_TX_INFO rfTxInfo;
RF_RX_INFO rfRxInfo;
uint8_t tx_buf[RF_MAX_PAYLOAD_SIZE];
uint8_t rx_buf[RF_MAX_PAYLOAD_SIZE];


void save_data(uint16_t tds, uint16_t temp);


void send_str_ping();
void send_data();
uint8_t check_reply(char *str, uint8_t timeoutms);
void erase_req();

uint16_t node_id = 0x0000;

int
main ()
{
	uint16_t i;

  nrk_setup_ports();
	nrk_setup_uart(UART_BAUDRATE_115K2);

  printf( "Starting up...\r\n" );

  nrk_init();
  nrk_time_set(0,0);

	numpoints = 0;
	/* Test code for 300-data-point download */
	/*
	for(i=0; i<300; i++){
		databank[i].cond = 234;
		databank[i].temp = 345;
	}
	numpoints = 300;
	*/

	comm_sem = nrk_sem_create(1,4);

	rf_rx_off();

  nrk_register_drivers();
  nrk_create_taskset ();
  nrk_start();

  return 0;
}


/* CHECK: Blocking on sem uses up reserve time? */
/* How else could the LED stop blinking? */

/* Sensor polling and data storage task */
void Task2()
{
	uint16_t i, j, tds, temp, tds_sum, temp_sum;
	int8_t fd, val;
	uint8_t led;
	nrk_time_t wait_time;

  // Open ADC device as read 
  fd = nrk_open(ADC_DEV_MANAGER,READ);
  if(fd == NRK_ERROR) 
		nrk_kprintf( PSTR("Failed to open ADC driver\r\n"));

	#define SWI_PIN_0 5
	#define SWI_PIN_1	6
	#define SENS_PWR  2
	#define TDS_PIN   4
	#define TEMP_PIN  3

	nrk_led_clr(ORANGE_LED);
	nrk_led_clr(RED_LED);
	nrk_led_clr(BLUE_LED);

	DDRF |= ((1 << SENS_PWR) | (1 << SWI_PIN_0) | (1 << SWI_PIN_1));
	PORTF &= ~((1 << SENS_PWR) | (1 << SWI_PIN_0) | (1 << SWI_PIN_1));

	while(1){

		led = GREEN_LED;

		/* Go back to sleep for 4 out of 5 wakeups */
		for(i=0; i<4; i++){
			nrk_led_set(led);
			for(j=0; j<20; j++){
				nrk_spin_wait_us(10000);
			}
			nrk_led_clr(led);
			nrk_wait_until_next_period();
		}
		
		nrk_sem_pend(comm_sem);

		PORTF |= (1 << SENS_PWR);

		wait_time.secs = 1;
		wait_time.nano_secs = 0;
		nrk_wait(wait_time);
		
		val = nrk_set_status(fd, ADC_CHAN, TEMP_PIN);
		if(val==NRK_ERROR) nrk_kprintf( PSTR("Failed to set ADC status\r\n" ));

		/* Take temperature readings */
		temp_sum = 0;
		printf("\r\ntemp: ");
		for(i=0; i<8; i++){
			val=nrk_read(fd, &temp, 2);
			temp_sum += temp;
			/*
			if(val==NRK_ERROR) nrk_kprintf( PSTR("Failed to read ADC\r\n" ));
			printf("%u ", temp);
			*/
			nrk_spin_wait_us(10000);
		}
		temp = temp_sum / 8;
		printf("(%u) ", temp);


		wait_time.secs = 1;
		wait_time.nano_secs = 0;
		nrk_wait(wait_time);

		/* Generate square-wave signal */
		for(i=0; i<2000; i++){
			nrk_spin_wait_us(500);
			PORTF |= (1 << SWI_PIN_0);
			PORTF &= ~(1 << SWI_PIN_1);

			nrk_spin_wait_us(500);
			PORTF |= (1 << SWI_PIN_1);
			PORTF &= ~(1 << SWI_PIN_0);
		}
		
		val = nrk_set_status(fd, ADC_CHAN, TDS_PIN);
		if(val==NRK_ERROR) nrk_kprintf( PSTR("Failed to set ADC status\r\n" ));

		/* Take conductivity readings */
		tds_sum = 0;
		printf("tds: ");
		for(i=0; i<8; i++){
			nrk_spin_wait_us(250);

			val=nrk_read(fd, &tds, 2);
			tds_sum += tds;
			/*
			if(val==NRK_ERROR) nrk_kprintf( PSTR("Failed to read ADC\r\n" ));
			printf("%u ", tds);
			*/

			nrk_spin_wait_us(250);
			PORTF |= (1 << SWI_PIN_0);
			PORTF &= ~(1 << SWI_PIN_1);

			nrk_spin_wait_us(500);
			PORTF |= (1 << SWI_PIN_1);
			PORTF &= ~(1 << SWI_PIN_0);
		}
		tds = tds_sum / 8;
		printf("(%u) ", tds);
		
		/* Save data, power down sensors, release sem, and sleep */
		save_data(tds, temp);
		printf("| numpoints: (%u)",numpoints);
		printf("\r\n");
		
		PORTF &= ~((1 << SENS_PWR) | (1 << SWI_PIN_0) | (1 << SWI_PIN_1));
		nrk_sem_post(comm_sem);
		nrk_wait_until_next_period();

	}

}


void save_data(uint16_t tds, uint16_t temp){
	if(numpoints < 2048){
		databank[numpoints].cond = tds;
		databank[numpoints].temp = temp;
		numpoints++;
	}
}

/* Communication task */
void Task1(){

	rfRxInfo.pPayload = rx_buf;
  rfRxInfo.max_length = RF_MAX_PAYLOAD_SIZE;
  rf_init (&rfRxInfo, 13, 0x2420, 0x1214);

	while(1){
		nrk_sem_pend(comm_sem);

		//rf_power_up();
		rf_rx_on();
		
		if(node_id < 10)
			sprintf((char *) tx_buf,"iden:00%u, points:%u",node_id,numpoints);
		else if(node_id < 100)
			sprintf((char *) tx_buf,"iden:0%u, points:%u",node_id,numpoints);
		else
			sprintf((char *) tx_buf,"iden:%u, points:%u",node_id,numpoints);
		send_str_ping();
	
		/* TODO: Check for two simultaneous dumps conflicting
			with each other */
		tx_buf[0] = 'd';
		tx_buf[1] = 'u';
		tx_buf[2] = 'm';
		tx_buf[3] = 'p';
		tx_buf[8] = '\0';

		putchar('*');
		/* Node only responds if it receives dump:[NODE_ID]
		** within 5 ms */
		if(check_reply((char *) tx_buf,5)){
			putchar('D');
			send_data();
			/* TODO: Add ID number to erase command,
			** possible data loss */
			if(check_reply("erase",20))
				erase_req();
		}
		rf_rx_off();
		//rf_power_down();

		nrk_sem_post(comm_sem);
		nrk_wait_until_next_period();
	}
}


/* Sends plain text over wireless:
**
** "start"
** "time:[TIME]"
**	"data:0 (123,123) (123,123) ...
**	"data:1 (123,123) (123,123) ...
**	...
**	end
*/
void send_data()
{
	uint8_t i, j;
	nrk_time_t cur_time;
	
	rfTxInfo.pPayload=tx_buf;
	rfTxInfo.destAddr = 0x1215;
	rfTxInfo.cca = 0;
	rfTxInfo.ackRequest = 1;

	sprintf((char *) tx_buf, "start");
  rfTxInfo.length= strlen((char *) tx_buf) + 1;
	for(i=0; i<20; i++){
		if(rf_tx_packet(&rfTxInfo) == 1)
			break;
	}

	nrk_time_get(&cur_time);
	sprintf((char *) tx_buf, "time:%lu", cur_time.secs);
  rfTxInfo.length= strlen((char *) tx_buf) + 1;
	for(i=0; i<20; i++){
		if(rf_tx_packet(&rfTxInfo) == 1)
			break;
	}


	for(j=0; j<(numpoints+7)/8; j++){	
		sprintf((char *) tx_buf, "data:%d ", j);
		for(i=0; i<8; i++){
			sprintf((char *) (tx_buf+strlen((char *) tx_buf)), 
					"(%d,%d)", databank[i+(8*j)].cond, databank[i+(8*j)].temp);
		}
		rfTxInfo.length= strlen((char *) tx_buf) + 1;
		for(i=0; i<20; i++){
			if(rf_tx_packet(&rfTxInfo) == 1)
				break;
		}
	}


	sprintf((char *) tx_buf, "end");
  rfTxInfo.length= strlen((char *) tx_buf) + 1;
	for(i=0; i<20; i++){
		if(rf_tx_packet(&rfTxInfo) == 1)
			break;
	}

	return;
}


/* Clears databank and sends an ACK */
void erase_req(){
	uint8_t i;
	
	memset(databank, 0, BANKSIZE*4);
	numpoints = 0;

	sprintf((char *) tx_buf, "memclr");
  rfTxInfo.length= strlen((char *) tx_buf) + 1;
	for(i=0; i<20; i++){
		if(rf_tx_packet(&rfTxInfo) == 1)
			break;
	}
}


/* Sends string as payload */
void send_str_ping()
{

	rfTxInfo.pPayload=tx_buf;
  rfTxInfo.length= strlen((char *) tx_buf) + 1;
	rfTxInfo.destAddr = 0x1215;
	rfTxInfo.cca = 0;
	rfTxInfo.ackRequest = 0;
	
	rf_tx_packet(&rfTxInfo);

	return;
}


/* Checks for reply string over wireless, times
** out after timeoutms milliseconds */
uint8_t check_reply(char *str, uint8_t timeoutms)
{
	nrk_time_t time, future_time, dummy_time;

	nrk_time_get(&future_time);
	future_time.nano_secs += timeoutms * NANOS_PER_MS;
	nrk_time_compact_nanos(&future_time);
	
	do{
		if(rf_polling_rx_packet() == 1){
			if(rfRxInfo.length == strlen(str)+1){
				if(rfRxInfo.pPayload[strlen(str)] == '\0'
						&& !strcmp((char *) rfRxInfo.pPayload, str)){
					return 1;
				}
			}
		}
		nrk_time_get(&time);
	}while(nrk_time_sub(&dummy_time, future_time, time) != NRK_ERROR);

	return 0;
}


void
nrk_create_taskset()
{
  TaskOne.task = Task1;
  nrk_task_set_stk( &TaskOne, Stack1, NRK_APP_STACKSIZE);
  TaskOne.prio = 1;
  TaskOne.FirstActivation = TRUE;
  TaskOne.Type = BASIC_TASK;
  TaskOne.SchType = PREEMPTIVE;
  TaskOne.period.secs = 0;
  TaskOne.period.nano_secs = 500 * NANOS_PER_MS;
  /*Check reservation */
	TaskOne.cpu_reserve.secs = 30;
  TaskOne.cpu_reserve.nano_secs = 0;
  TaskOne.offset.secs = 0;
  TaskOne.offset.nano_secs= 0;
  nrk_activate_task (&TaskOne);

 	nrk_task_set_entry_function( &TaskTwo, Task2);
  nrk_task_set_stk( &TaskTwo, Stack2, NRK_APP_STACKSIZE);
  TaskTwo.prio = 2;
  TaskTwo.FirstActivation = TRUE;
  TaskTwo.Type = BASIC_TASK;
  TaskTwo.SchType = PREEMPTIVE;
  TaskTwo.period.secs = 60;
  TaskTwo.period.nano_secs = 0;
  TaskTwo.cpu_reserve.secs = 30;
  TaskTwo.cpu_reserve.nano_secs = 0;
  TaskTwo.offset.secs = 0;
  TaskTwo.offset.nano_secs= 0;
  nrk_activate_task (&TaskTwo);
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


