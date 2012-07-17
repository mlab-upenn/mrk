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
uint16_t datastart;
nrk_sem_t *comm_sem;

nrk_time_t current_cmp_time, future_cmp_time, dummy_time;
uint8_t cmpbuf[16];


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

void append_nodeid(char *buf);
void send_str(uint8_t ack);
uint8_t check_reply(char *str, uint8_t timeoutms);
void set_timeout(uint8_t timeoutms);

uint16_t nodeid = 0x0000;

int main ()
{
	uint16_t i;

  nrk_setup_ports();
	nrk_setup_uart(UART_BAUDRATE_115K2);

  printf( "Starting up...\r\n" );

  nrk_init();
  nrk_time_set(0,0);

	numpoints = 0;
	datastart = 0;

	/* Test code for 1000-data-point download */
	/*	
	for(i=0; i<3072; i++){
		save_data(i,i);
	}
	*/

	comm_sem = nrk_sem_create(1,4);

	rf_rx_off();
	rf_power_down();

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

	// Open ADC device as read
	fd = nrk_open(ADC_DEV_MANAGER,READ);
  	if(fd == NRK_ERROR) 
		nrk_kprintf( PSTR("Failed to open ADC driver\r\n"));

	while(1){

		led = GREEN_LED;

		/* Go back to sleep for 4 out of 5 wakeups */
		for(i=0; i<9; i++){
			nrk_led_set(led);
			for(j=0; j<20; j++){
				nrk_spin_wait_us(10000);
			}
			nrk_led_clr(led);

			ADCSRA &= ~(1 << ADEN);
			nrk_wait_until_next_period();
		}
		
		nrk_sem_pend(comm_sem);

  		PORTF |= (1 << SENS_PWR);

		wait_time.secs = 1;
		wait_time.nano_secs = 0;
		nrk_wait(wait_time);
		ADCSRA |= (1 << ADEN);
		
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

		ADCSRA &= ~(1 << ADEN);
		wait_time.secs = 1;
		wait_time.nano_secs = 0;
		nrk_wait(wait_time);
		ADCSRA |= (1 << ADEN);
		
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

		ADCSRA &= ~(1 << ADEN);	
		
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
	else if(numpoints == 2048){
		databank[datastart].cond = tds;
		databank[datastart].temp = temp;
		datastart++;
		if(datastart == 2048)
			datastart = 0;
	}
}


/* Communication task */
void Task1(){
	
	char c;
	uint8_t pg, i, j;

	rf_power_up();
	rfRxInfo.pPayload = rx_buf;
  	rfRxInfo.max_length = RF_MAX_PAYLOAD_SIZE;
  	rf_init (&rfRxInfo, 13, 0x2420, 0x1214);

	while(1){
		nrk_sem_pend(comm_sem);

		rf_power_up();
		rf_rx_on();
	
		sprintf((char *) tx_buf,"sp,id");
		append_nodeid((char *) tx_buf);
		sprintf((char *) tx_buf,"%s,dc%d;", tx_buf, numpoints);
		send_str(0);

		strncpy((char *) cmpbuf, (char *) tx_buf, 9);
		cmpbuf[0] = 'm';
		cmpbuf[9] = '\0';

		putchar('*');
		/* Node stays awake if it receives wake:[NODEID]
		** within 5 ms */
		if(check_reply((char *) cmpbuf, 5)){
			putchar('W');
			
			set_timeout(50);	
			do{
				if(rf_polling_rx_packet() == 1){
					c = rfRxInfo.pPayload[1];
					rfRxInfo.pPayload[1] = '*';
					cmpbuf[1] = '*';
					if(!strncmp(cmpbuf, rfRxInfo.pPayload, strlen(cmpbuf))){		
						
						switch(c){
							case 'i':	
								nrk_time_get(&current_cmp_time);
								sprintf((char *) tx_buf,"si,id");
								append_nodeid((char *) tx_buf);
								sprintf((char *) tx_buf,"%s,dc%d,st%d,tm%ld;"
										, tx_buf, numpoints, datastart,
										current_cmp_time.secs);
								send_str(1);
								set_timeout(50);
								break;

							case 'd':
								pg = atoi(((char *) rfRxInfo.pPayload) + 12);
								
								for(i=0; i<8; i++){
									sprintf((char *) tx_buf,"sd,id");
									append_nodeid((char *) tx_buf);
									sprintf((char *) tx_buf,"%s,pg%d,pt%d,", 
											tx_buf, pg, i);
									for(j=0; j<8; j++){
										sprintf((char *) tx_buf, "%s%d,%d,",
												tx_buf, databank[(64*pg)+(8*i)+j].cond,
												databank[(64*pg)+(8*i)+j].temp);
									}
									tx_buf[strlen(tx_buf)-1] = ';';
									send_str(1);
								}
								set_timeout(200);
								break;

							case 'e':		
								memset(databank, 0, BANKSIZE*4);
								numpoints = 0;
								datastart = 0;

								sprintf((char *) tx_buf,"se,id");
								append_nodeid((char *) tx_buf);
								sprintf((char *) tx_buf,"%s;", tx_buf);
								send_str(1);
								set_timeout(50);
								break;

						}
					}
				}
				nrk_time_get(&current_cmp_time);
			}while(nrk_time_sub(&dummy_time, future_cmp_time, 
					current_cmp_time) != NRK_ERROR);

		}

		rf_rx_off();
		rf_power_down();
		
		nrk_sem_post(comm_sem);
		nrk_wait_until_next_period();
	}
}


void append_nodeid(char *buf)
{
		if(nodeid < 0x1000)
			sprintf((char *) tx_buf, "%s0", tx_buf);
		if(nodeid < 0x100)
			sprintf((char *) tx_buf, "%s0", tx_buf);
		if(nodeid < 0x10)
			sprintf((char *) tx_buf, "%s0", tx_buf);
		sprintf((char *) tx_buf, "%s%X", tx_buf, nodeid);
}


/* Sends string as payload */
void send_str(uint8_t ack)
{
	uint8_t i, retry_count;

	rfTxInfo.pPayload=tx_buf;
  rfTxInfo.length= strlen((char *) tx_buf) + 1;
	rfTxInfo.destAddr = 0x1215;
	rfTxInfo.cca = 0;
	rfTxInfo.ackRequest = ack ? 1 : 0;

	retry_count = ack ? 20 : 1;
	for(i=0; i<retry_count; i++){
		if(rf_tx_packet(&rfTxInfo) == 1)
			break;
	}
	return;
}


/* Checks for reply string over wireless, times
** out after timeoutms milliseconds */
uint8_t check_reply(char *str, uint8_t timeoutms)
{

	set_timeout(timeoutms);
	do{
		if(rf_polling_rx_packet() == 1){
			if(!strncmp(str, rfRxInfo.pPayload, strlen(str))){		
					return 1;
			}
		}
		nrk_time_get(&current_cmp_time);
	}while(nrk_time_sub(&dummy_time, future_cmp_time, current_cmp_time) != NRK_ERROR);

	return 0;
}


void set_timeout(uint8_t timeoutms)
{	
	nrk_time_get(&future_cmp_time);
	future_cmp_time.nano_secs += timeoutms * NANOS_PER_MS;
	nrk_time_compact_nanos(&future_cmp_time);
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
	TaskOne.cpu_reserve.secs = 60;
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


