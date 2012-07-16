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
#include <nrk_stack_check.h>
#include <basic_rf.h>

#include <nrk_driver_list.h>
#include <nrk_driver.h>
#include <slip.h>
#include <ade7878.h>


// Add this #define if using the i2c adapter board. 
// This will swap the interrupt pin so that i2c is free
#define I2C_ADAPTER_INSTALLED


#define C_CircBuffSize	2046		//Make sure it is a multiple of 6 since we write 6 bytes at a time
#define C_SlipPktSize	90			//Make sure it is a multiple of 6. And safely keep it < 95
#define C_ThresholdPwr 20000		//Threshold raw value of power difference for checking if evetn occ. 40000 -> approx 15 watts

NRK_STK Stack1[NRK_APP_STACKSIZE];	
nrk_task_type TaskOne;
void Task1(void);						// To initialize timer

NRK_STK Stack2[NRK_APP_STACKSIZE];
nrk_task_type TaskTwo;
void Task2 (void);					// Slow display of RMS values

NRK_STK Stack3[NRK_APP_STACKSIZE];
nrk_task_type TaskThree;
void Task3 (void);					//	Does nothing

NRK_STK Stack4[NRK_APP_STACKSIZE];
nrk_task_type TaskFour;
void Task4 (void);					// Checks if buffer has enough fast data, and then does a SLIP tx

NRK_STK Stack5[NRK_APP_STACKSIZE];
nrk_task_type TaskFive;
void Task5 (void);					// Event Detection

void nrk_create_taskset();
//ADE7878 related............................
uint8_t F_ChkIntEntry_U8R, V_RdLcycmode_U8R;

uint32_t V_BasicRdChk_U32R, V_Status1RdWr_U32R;  
uint32_t V_ArmsCurr_U32R, V_ArmsVolt_U32R ;
int32_t V_ArmsWatt_S32R;

int32_t V_Awatt_S32R=0,V_Avar_S32R=0;
uint8_t F_ReadyForSignal_U8R=1,F_1secData_U8R=0 ;
uint16_t V_RdDSP_U16R; 
uint8_t F_RdDSP_U8R = 0;
uint32_t V_DSPrstCnt_U32R = 0;
int32_t V_RdParam1_S32R = 0, V_RdParam2_S32R = 0, V_RdParam3_S32R = 0;
uint32_t V_CalcTemp1_U32R,V_CalcTemp2_U32R;
int32_t V_CalcTemp1_S32R,V_CalcTemp2_S32R;

////////////////////////////////
uint32_t V_DataOld_U32R = 0, V_DataNew_U32R = 0, V_EvnDetCntr_U32R = 0;
int32_t V_DiffData_S32R;
uint8_t F_Det_U8R = 0, F_Occ_U8R = 0;

uint8_t V_DummyCnt_U32R=0,A_1secRmsVals_U8R[20],F_1secDataReady_U8R=0,V_1secDummyCnt_U32R=0;
uint32_t V_Dummy32a_U32R=0,V_Dummy32b_U32R=0;
uint32_t V_TINTcnt_U32R=0;
///////////////////////////////
//End of ADE7878.............................

//RF realted..................................
void my_callback(uint16_t global_slot );
RF_TX_INFO rfTxInfo;
RF_RX_INFO rfRxInfo;
uint8_t tx_buf[RF_MAX_PAYLOAD_SIZE];
uint8_t rx_buf[RF_MAX_PAYLOAD_SIZE];
uint8_t tx_size;
//End of RF realted..........................

//Newly added in thie code
//nrk_sig_t signal_one;
int8_t V_BuffCnt_U8R=0;
//End of newly added

//SLIP related
uint8_t V_TxBuff_U8R[2048];
uint8_t V_WrSeqNo_U8R = 0;
uint32_t V_WrPtr_U32R = 0, V_RdPtr_U32R = 0;
uint8_t F_AllowRead_U8R = 0;
//End of SLIP related

int
main ()
{
	uint8_t t;
	uint8_t cnt,i,j,length;
	uint32_t dcnt = 0;

	//Initializig ports
	nrk_setup_ports(); 
   nrk_setup_uart (UART_BAUDRATE_115K2);
	nrk_kprintf( PSTR("Starting up...\r\n") );
	nrk_led_clr(RED_LED);nrk_led_clr(BLUE_LED);nrk_led_clr(ORANGE_LED);nrk_led_clr(GREEN_LED); 
	
	nrk_led_set(BLUE_LED);
	//Initializing RF
	rfRxInfo.pPayload = rx_buf;
   rfRxInfo.max_length = RF_MAX_PAYLOAD_SIZE;
	nrk_int_enable();	 
   rf_init (&rfRxInfo, 13, 0x2420, 0x1214);
	
	//Wireless tx "Restart" upto starting
	sprintf(tx_buf,"\r\nRestart");
	rfTxInfo.pPayload=tx_buf;
	rfTxInfo.length= strlen(tx_buf)+1;
	rfTxInfo.destAddr = 0x1215;
	rfTxInfo.cca = 0;
	rfTxInfo.ackRequest = 0;
	if(rf_tx_packet(&rfTxInfo) != 1)
		printf("--- RF_TX ERROR ---\r\n");	

	DDRD |= _BV(PORTD3);		//ADE7878 Reset line 

	//Interrupt initialization for ADE7878
	sei();						//Enable Global interrupts
#ifndef I2C_ADAPTER_INSTALLED
	DDRD &= ~(_BV(PORTD1));	//Making INT1 pin as input
#else
	DDRD &= ~(_BV(PORTD3));	//Making INT3 pin as input
#endif

	EIMSK = 0x00;				//Mask all
	EICRA = 0x08;				//EINT1 Falling Edge - changed for new FF

	PORTD |= _BV(PORTD3);	//Reset ADE7878
	//Set ADE in normal power mode PM0=1 (PE2), PM1=0 (PE3)
	printf("delay11111111111111111111111111"); //delay - may not be needed now, was added when I was debugging some other issue
	PORTD &= ~_BV(PORTD3);
	//Set ADE in normal power mode PM0=1 (PE2), PM1=0 (PE3)
	DDRE 	|= _BV(PORTE2) | _BV(PORTE3);
	PORTE |= _BV(PORTE2);
	PORTE &= ~(_BV(PORTE3));

	//These bunch of lines below may not be needed since they have- added it while debugging a recent issue, did not try the code without these lines
#ifndef I2C_ADAPTER_INSTALLED
	EIMSK = 0x02;	//Unmask EINT1
#else
	EIMSK = 0x08;	//Unmask EINT3
#endif
	printf("delay22222222222222222222222222");
	PORTD |= _BV(PORTD3);
	PORTE |= _BV(PORTE2);
	PORTE &= ~(_BV(PORTE3));

	//Check if the interrupt really executed or not
	while(EIMSK !=0);

	//Enable SPI Master, Set Clock rate fck/4
	SPCR = 0x01;
	SPCR |= _BV(SPE) | _BV(MSTR) | _BV(CPOL) | _BV(CPHA);
	
	PORTB |= _BV(PORTB0);//default state of SS should be low

	//This segment below is added because it was observed that the 7878
	//SPI or the ATMEL SPI takes some time to start. So waiting till
	//some register gives its default value
	printf("Wait..");
	V_RdLcycmode_U8R=0;
	while(V_RdLcycmode_U8R!=0x78)
		V_RdLcycmode_U8R = ade_read8(LCYCMODE);
	printf("\r\n7878 Ready %x", V_RdLcycmode_U8R);

	//Initialize ADE7878 DSP and a few other registers
	if(ade_init() < 0)
		printf("\nInit failed");
	else
		printf("\nInit Success");

	//Read of this register - just for checking
	V_RdLcycmode_U8R = ade_read8(LCYCMODE);
	printf("\r\nLCYC =  %x", V_RdLcycmode_U8R);

	//Read the INT Status register to clear the interrupt
	V_Status1RdWr_U32R = ade_read32(STATUS1);
	ade_write32(STATUS1, V_Status1RdWr_U32R);
	
	//Initialize NRK	
	nrk_init();
  	nrk_time_set(0,0);
	nrk_create_taskset ();
  	nrk_start();
  	return 0;
}

//This timer interrupt occurs every 1ms. We read the Watt, VAR registers and write to buffer
void my_timer_callback()
{
	int8_t i,v,dummy_buf[10];
//	nrk_led_toggle(GREEN_LED);
	
	//V_RdParam1_S32R	=	ade_read32(AWATT);
	//V_RdParam2_S32R		=	ade_read32(AVAR);
	//V_Awatt_S32R = ade_read32(AWATT);
	V_RdParam1_S32R = ade_read32(IAWV);
	V_RdParam2_S32R = ade_read32(VAWV);
	
	//nrk_led_set(RED_LED);
	
	//Write the 6 new bytes to the buffer and increment Wr pointer
	memcpy(&V_TxBuff_U8R[V_WrPtr_U32R],&V_RdParam1_S32R,3);
	memcpy(&V_TxBuff_U8R[V_WrPtr_U32R+3],&V_RdParam2_S32R,3);
	V_WrPtr_U32R+=6;
	
	//If Wr Ptr reached end-of-buffer, set a flag to let the Rd pointer know!
	if(V_WrPtr_U32R == C_CircBuffSize)
	{	
		V_WrPtr_U32R = 0;
		V_WrSeqNo_U8R = 1;
	}

	//Check if there is enough data to be read and set flag
	if(((V_WrSeqNo_U8R*C_CircBuffSize + V_WrPtr_U32R)-(V_RdPtr_U32R)) >= C_SlipPktSize)
	{
		F_AllowRead_U8R = 1;
	}
	
	//Every 1 second, we read RMS values. This is done inside the ISR and not as a separate task because both - the ISR and the lines below access the 7878 through SPI and we cant let one SPI instr interrupt another SPI instr
	V_TINTcnt_U32R++;
	if(V_TINTcnt_U32R == 500)
	{
		F_1secDataReady_U8R=1;
		V_ArmsCurr_U32R = ade_read32(AIRMS);
		V_ArmsVolt_U32R = ade_read32(AVRMS);
		V_ArmsVolt_U32R = V_ArmsVolt_U32R*120;
		V_ArmsVolt_U32R = V_ArmsVolt_U32R/3930000;
		V_ArmsCurr_U32R = V_ArmsCurr_U32R*270;
		V_ArmsCurr_U32R = V_ArmsCurr_U32R/126000;//new CT, 20ohms
	}
	else if(V_TINTcnt_U32R == 1000)
	{//every 1 second I am also checking for DSP reset, but I didn't want to do it in the same iteration that I read the 7878 regs, just to distribute the SPI access time
		V_RdDSP_U16R = ade_read16(RUN);
		if(V_RdDSP_U16R == 0)
		{
			V_DSPrstCnt_U32R++;
			ade_write16(RUN,START);//if the DSP got reset, start it so that it continues to run
		}
		V_TINTcnt_U32R = 0;
	}

	//nrk_led_clr(GREEN_LED);
}

//This task initializes the Timer
void Task1()
{
	uint16_t cnt;
	uint8_t val;

	printf( "My node's address is %d\r\n",NODE_ADDR );

  	printf( "Task1 PID=%d\r\n",nrk_get_pid());
  	cnt=0;

  	// Setup application timer with:
  	//       Prescaler = 2 
  	//       Compare Match = 2000
  	//       Sys Clock = 16000 MHz
  	// Prescaler 2 means divide sys clock by 8
  	// 16000000 / 8 = 2MHz clock
  	// 1 / 2MHz = 0.5 us per tick
  	// 0.5 us * 2000 = 1 ms / per interrupt callback

  	val=nrk_timer_int_configure(NRK_APP_TIMER_0, 2, 2000, &my_timer_callback );//value 2000 is for 1ms. Change to 16000 for 8ms
  	if(val==NRK_OK) nrk_kprintf( PSTR("Callback timer setup\r\n"));
  	else nrk_kprintf( PSTR("Error setting up timer callback\r\n"));

  	// Zero the timer
  	nrk_timer_int_reset(NRK_APP_TIMER_0);
  	// Start the timer
  	nrk_timer_int_start(NRK_APP_TIMER_0);

  	while(1) 
  	{
		nrk_wait_until_next_period();
	}
}

// Slow display of RMS values
// This task runs every 100ms and checks if RMS data is available to be printed.
// Actually, it should be enough to call this every 1 second. But the data goes through the RF tx, to the other node RF Rx, and then through RS232 to the SLIP Server on PC, and I observed that the SLIP Server sometimes missed the display of a few seconds data, so doing this every 100ms.
void Task2()
{
	uint8_t cnt;
	printf( "Task2 PID=%d\r\n",nrk_get_pid());
	cnt=0;
  
	while(1) 
	{
		//nrk_led_toggle(GREEN_LED);
		nrk_led_clr(BLUE_LED);
		nrk_led_toggle(RED_LED);	//since micro FF has only RED
		if(F_1secDataReady_U8R==1)	//Check is the RMS data is available from the 1ms second timer
		{
			F_1secDataReady_U8R=0;	//Clear flag to avoid repeat of same data
			printf("\r\nI=%ldmA  ",V_ArmsCurr_U32R);
			printf("V=%ldV",V_ArmsVolt_U32R);
	//		sprintf(tx_buf,"\r\nI = %ld, V = %ld, P = %ld",V_ArmsCurr_U32R, V_ArmsVolt_U32R, V_ArmsWatt_S32R);	//Write the RMS V, I into the RF tx buffer
			sprintf(tx_buf,"\r\nI %ldmA, V %ld, N %ld + ",V_ArmsCurr_U32R, V_ArmsVolt_U32R, V_DSPrstCnt_U32R);	//Write the RMS V, I into the RF tx buffer
			
			//Transmit the data through RF
			rfTxInfo.pPayload=tx_buf;
			rfTxInfo.length= strlen(tx_buf)+1;
			rfTxInfo.destAddr = 0x1215;
			rfTxInfo.cca = 0;
			rfTxInfo.ackRequest = 0;
			if(rf_tx_packet(&rfTxInfo) != 1)
				printf("--- RF_TX ERROR ---\r\n");
			cnt++;			
		}
		nrk_led_set(BLUE_LED);
		nrk_wait_until_next_period();
		//cnt++;
	}
}

//Right now this task is not used an disabled
void Task3()
{
	uint16_t cnt;
	uint16_t i;
  	printf( "Task3 PID=%d\r\n",nrk_get_pid());
  	cnt=0;
  	while(1) {
			printf( "Task3 cnt=%d\r\n",cnt );
		nrk_wait_until_next_period();
		cnt++;
	}
}

//Checks if buffer has enough fast data, and then does a SLIP tx
void Task4()
{
	uint8_t cnt,v_BufLen_u8r=0;

	printf( "Task4 PID=%d\r\n",nrk_get_pid());
	cnt=0;

	while(1) 
	{
		nrk_led_toggle(ORANGE_LED);
		nrk_led_clr(BLUE_LED);
		if(F_AllowRead_U8R==1)		//If there is enough data to do a SLIP tx. This flag is set by the 1ms timer interrupt
		{
			F_AllowRead_U8R = 0; 	//Clear flag to avoid repeating same data
			
			//The slip tx function encapsulates the data as a slip packet and puts it in tx_buf
			tx_size = slip_tx(&V_TxBuff_U8R[V_RdPtr_U32R],C_SlipPktSize,&V_TxBuff_U8R[C_CircBuffSize-1],C_CircBuffSize,tx_buf);
			tx_buf[tx_size]=0;			//Write a NULL at the end - This may not be needed
			v_BufLen_u8r = tx_size+1;	//Length including the NULL

			V_RdPtr_U32R=V_RdPtr_U32R+C_SlipPktSize;	//Now that we just read data (in the slip_tx func), increment the Rd ptr
			if(V_RdPtr_U32R>=C_CircBuffSize)				//If end-of-buffer reached, roll over the pointer
			{
				V_RdPtr_U32R = V_RdPtr_U32R - C_CircBuffSize;
				V_WrSeqNo_U8R = 0;							//This ptr is set when Wr rolls over and Rd still not rolled over. Now that read too rolled over, clr this flag
			}

			//Transmit the data through RF
			rfTxInfo.pPayload=tx_buf;
			rfTxInfo.length= v_BufLen_u8r;
			rfTxInfo.destAddr = 0x1215;
			rfTxInfo.cca = 0;
			rfTxInfo.ackRequest = 0;
			if(rf_tx_packet(&rfTxInfo) != 1)
			printf("--- RF_TX ERROR ---\r\n");
			//	for(i=0; i<4; i++ )
			//		halWait(6000);
		}
		nrk_led_set(BLUE_LED);
		nrk_wait_until_next_period();
		//cnt++;
	}
}

//Event Detection
//We acquire fast data every 1ms. But it is enough if we check for every every 200ms. So this task has a period of 200ms
//We maintain 2 flags in this routine: F_Det_U8R indicates the first occurrence of an event. F_Occ_U8R indicates the first or continued occurrence of an event
void Task5()
{
	uint16_t cnt;
	uint16_t i;
	printf( "Task5 PID=%d\r\n",nrk_get_pid());
	cnt=0;
	while(1) 
	{
/*		V_DataNew_U32R = V_Awatt_S32R;
		V_DiffData_S32R = V_DataNew_U32R - V_DataOld_U32R;	//Find the difference between the current data and the data that was got 200ms ago
		V_DataOld_U32R = V_DataNew_U32R;

		if((V_DiffData_S32R<C_ThresholdPwr)&&(V_DiffData_S32R>(-1*C_ThresholdPwr)))	//Check if this difference is greater than a threshold
//		if((V_ArmsCurr_U32R%10)==0) //- for testing purpose	//temp - remove and uncomment prev line when 7878 works fine
		{
			//If no change in power, reset both flags
			F_Occ_U8R = 0;	
			F_Det_U8R = 0;
		}
		else
		{
			if(F_Occ_U8R==0)
			{
				//If there is a change and if the event was not already occurring, set both flags
				F_Det_U8R = 1;
				F_Occ_U8R = 1;
			}
			else	//If there is a change and the event was already occuring, this is jus the continued occurence - means the power is chinaign slowly for more than 200ms, so dont mark it as a new event
				F_Det_U8R = 0;
		}

		if(F_Det_U8R==1)	//If an event was detected, send out an "Event" packet through RF
		{
			printf("\r\nEvent!");
			sprintf(tx_buf,"\r\nEvent");
			rfTxInfo.pPayload=tx_buf;
			rfTxInfo.length= strlen(tx_buf)+1;
			rfTxInfo.destAddr = 0x1215;
			rfTxInfo.cca = 0;
			rfTxInfo.ackRequest = 0;
			nrk_gpio_set(NRK_DEBUG_0);
			if(rf_tx_packet(&rfTxInfo) != 1)
				printf("--- RF_TX ERROR ---\r\n");	
		}
*/		//	V_RdDSP_U16R = ade_read16(RUN);	//temp - uncomment this secion once the 7878 works fine
		//V_RdDSP_U16R = ade_read16(APHCAL);
	/*	if(F_RdDSP_U8R == 1)
		{
			F_RdDSP_U8R = 0;
			if(V_RdDSP_U16R==0)
			{
				//	ade_write16(RUN, START);
				nrk_led_set(RED_LED);
				V_
				//printf("\r\n$$");
			}
		} */
		nrk_wait_until_next_period();
		cnt++;
	}
}

//This external interrupt is sent by the 7878 when we reset it


#ifndef I2C_ADAPTER_INSTALLED
ISR(INT1_vect)
#else
ISR(INT3_vect)
#endif
{
	uint8_t i;

//	nrk_led_set(GREEN_LED);
	EIMSK = 0x00;
//	printf("INT occ");
	//nrk_led_set(RED_LED);
	//Set MOSI and SCK as output, others as input
	DDRB|=  _BV(PORTB0) | _BV(PORTB1) | _BV(PORTB2);
	DDRB&=  ~(_BV(PORTB3));
	PORTB|= _BV(PORTB0) | _BV(PORTB1) | _BV(PORTB2);

	
	for(i=0; i<200; i++)
	{
		PORTB ^= _BV(PORTB0);	//We need to toggle this multiple times (>3) to set the SPI mode of communication
	//	nrk_led_toggle(GREEN_LED);
	}

	F_ChkIntEntry_U8R = 1;
}

void
nrk_create_taskset()
{
  TaskOne.task = Task1;					// To initialize timer
  TaskOne.Ptos = (void *) &Stack1[NRK_APP_STACKSIZE];
  TaskOne.Pbos = (void *) &Stack1[0];
  TaskOne.prio = 2;
  TaskOne.FirstActivation = TRUE;
  TaskOne.Type = BASIC_TASK;
  TaskOne.SchType = PREEMPTIVE;
  TaskOne.period.secs = 0;
  TaskOne.period.nano_secs = 250*NANOS_PER_MS;
  TaskOne.cpu_reserve.secs = 0;
  TaskOne.cpu_reserve.nano_secs =  50*NANOS_PER_MS;
  TaskOne.offset.secs = 0;
  TaskOne.offset.nano_secs= 0;
  nrk_activate_task (&TaskOne);

  TaskTwo.task = Task2;					// Slow display of RMS values
  TaskTwo.Ptos = (void *) &Stack2[NRK_APP_STACKSIZE];
  TaskTwo.Pbos = (void *) &Stack2[0];
  TaskTwo.prio = 2;
  TaskTwo.FirstActivation = TRUE;
  TaskTwo.Type = BASIC_TASK;
  TaskTwo.SchType = PREEMPTIVE;
  TaskTwo.period.secs = 0;
  TaskTwo.period.nano_secs = 100*NANOS_PER_MS;
  TaskTwo.cpu_reserve.secs = 0;
  TaskTwo.cpu_reserve.nano_secs = 30*NANOS_PER_MS;
  TaskTwo.offset.secs = 0;
  TaskTwo.offset.nano_secs= 0;
  nrk_activate_task (&TaskTwo);


  TaskThree.task = Task3;			//	Does nothing
  TaskThree.Ptos = (void *) &Stack3[NRK_APP_STACKSIZE];
  TaskThree.Pbos = (void *) &Stack3[0];
  TaskThree.prio = 3;
  TaskThree.FirstActivation = TRUE;
  TaskThree.Type = BASIC_TASK;
  TaskThree.SchType = PREEMPTIVE;
  TaskThree.period.secs = 1;
  TaskThree.period.nano_secs = 0;
  TaskThree.cpu_reserve.secs = 0;
  TaskThree.cpu_reserve.nano_secs = 100*NANOS_PER_MS;
  TaskThree.offset.secs = 0;
  TaskThree.offset.nano_secs= 0;
//  nrk_activate_task (&TaskThree);

  TaskFour.task = Task4;					// Checks if buffer has enough fast data, and then does a SLIP tx
  TaskFour.Ptos = (void *) &Stack4[NRK_APP_STACKSIZE];
  TaskFour.Pbos = (void *) &Stack4[0];
  TaskFour.prio = 1;
  TaskFour.FirstActivation = TRUE;
  TaskFour.Type = BASIC_TASK;
  TaskFour.SchType = PREEMPTIVE;
  TaskFour.period.secs = 0;
  TaskFour.period.nano_secs = 15*NANOS_PER_MS;
  TaskFour.cpu_reserve.secs = 0;
  TaskFour.cpu_reserve.nano_secs = 15*NANOS_PER_MS;
  TaskFour.offset.secs = 0;
  TaskFour.offset.nano_secs= 0;
  nrk_activate_task (&TaskFour);

  TaskFive.task = Task5;						//Event Detection
  TaskFive.Ptos = (void *) &Stack5[NRK_APP_STACKSIZE];
  TaskFive.Pbos = (void *) &Stack5[0];
  TaskFive.prio = 5;
  TaskFive.FirstActivation = TRUE;
  TaskFive.Type = BASIC_TASK;
  TaskFive.SchType = PREEMPTIVE;
  TaskFive.period.secs = 0;
  TaskFive.period.nano_secs = 200*NANOS_PER_MS;
  TaskFive.cpu_reserve.secs = 0;
  TaskFive.cpu_reserve.nano_secs = 100*NANOS_PER_MS;
  TaskFive.offset.secs = 0;
  TaskFive.offset.nano_secs= 0;
//  nrk_activate_task (&TaskFive);
}

void my_callback(uint16_t global_slot )
{
		static uint16_t cnt;

		printf( "callback %d %d\n",global_slot,cnt );
		cnt++;
}



/**
 *  This is a callback if you require immediate response to a packet
 */
RF_RX_INFO *rf_rx_callback (RF_RX_INFO * pRRI)
{
    // Any code here gets called the instant a packet is received from the interrupt   
    return pRRI;
}


