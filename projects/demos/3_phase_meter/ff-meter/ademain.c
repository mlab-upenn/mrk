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

#include <nrk_driver_list.h>
#include <nrk_driver.h>
#include <ff_basic_sensor.h>
#include <ade7878.h>
#include <slip.h>

#define C_CircBuffSize	2046
#define C_SlipPktSize	90

#define Fs					50
#define C_ThresholdPwr	40000

uint8_t F_ChkIntEntry_U8R ;

//ADE7878 Related variables
uint32_t V_BasicRdChk_U32R, V_Status1RdWr_U32R;  
uint32_t V_ArmsCurr_U32R, V_ArmsVolt_U32R ;
int32_t V_ArmsWatt_S32R;

int32_t V_Awatt_S32R=0,V_Avar_S32R=0;
//uint32_t V_AwattCalc1_U32R,V_AwattCalc2_U32R; 
uint8_t F_ReadyForSignal_U8R=1,F_1secData_U8R=0 ;
uint16_t V_RdDSP_U16R; 

uint32_t V_CalcTemp1_U32R,V_CalcTemp2_U32R;
int32_t V_CalcTemp1_S32R,V_CalcTemp2_S32R;

////////////////////////////////
uint32_t V_DataOld_U32R = 0, V_DataNew_U32R = 0, V_EvnDetCntr_U32R = 0;
int32_t V_DiffData_S32R;
uint8_t F_Det_U8R = 0, F_Occ_U8R = 0;


///////////////////////////////
NRK_STK Stack1[NRK_APP_STACKSIZE];
nrk_task_type TaskOne;
void Task1(void);

NRK_STK Stack2[NRK_APP_STACKSIZE];
nrk_task_type TaskTwo;
void Task2 (void);

NRK_STK Stack3[NRK_APP_STACKSIZE];
nrk_task_type TaskThree;
void Task3 (void);

NRK_STK Stack4[NRK_APP_STACKSIZE];
nrk_task_type TaskFour;
void Task4 (void);

NRK_STK Stack5[NRK_APP_STACKSIZE];
nrk_task_type TaskFive;
void Task5 (void);

void nrk_create_taskset();

//Signal related variables
uint8_t V_5Sec_U8R=0,V_TINTcnt_U8R=0;
nrk_sig_t signal_one;

//SLIP related variables
uint8_t V_TxBuff_U8R[2048];
uint8_t V_WrSeqNo_U8R=0; 
uint32_t V_WrPtr_U32R=0, V_RdPtr_U32R=0;

int main ()
{
	uint8_t v_TempCnt_u8r, v_Dummy_u8r, v_RdLcycmode_u8r;
  
	nrk_setup_ports();
	nrk_setup_uart(UART_BAUDRATE_115K2);
  
	nrk_kprintf( PSTR("Starting up...\r\n") );

	//////////////////////////////////////////////////
	F_ChkIntEntry_U8R = 0; 
	nrk_led_clr(BLUE_LED);
	nrk_led_clr(GREEN_LED);
	nrk_led_clr(ORANGE_LED);
	nrk_led_clr(RED_LED);

	for(v_TempCnt_u8r=0;v_TempCnt_u8r<100;v_TempCnt_u8r++)
		V_TxBuff_U8R[v_TempCnt_u8r]=v_TempCnt_u8r+'a';
	V_RdPtr_U32R = 0;
	slip_tx(&V_TxBuff_U8R[V_RdPtr_U32R],C_SlipPktSize,&V_TxBuff_U8R[C_CircBuffSize-1],C_CircBuffSize);

	signal_one=nrk_signal_create();
	
	sei();
	DDRD &= ~(_BV(PORTD3));	//Making INT3 pin as input
	EIMSK = 0x00;
	EICRA = 0x80;
	EIMSK = 0x08;

	printf("#");
	//Set ADE in normal power mode PM0=1 (PE2), PM!=0 (PE3)
	DDRE 	|= _BV(PORTE2) | _BV(PORTE3);
	PORTE |= _BV(PORTE2);
	PORTE &= ~(_BV(PORTE3));

	while(F_ChkIntEntry_U8R==0);
	while(EIMSK !=0);

	//Enable SPI Master, Set Clock rate fck/4
	SPCR = 0x01;
	SPCR |= _BV(SPE) | _BV(MSTR) | _BV(CPOL) | _BV(CPHA);
	
	PORTB |= _BV(PORTB0);//default state of SS should be low
	v_RdLcycmode_u8r=0;

	//This segment below is added because it was observed that the 7878
	//SPI or the ATMEL SPI takes some time to start. So waiting till
	//some register gives its default value
	printf("Wait..");
	while(v_RdLcycmode_u8r!=0x78)
		v_RdLcycmode_u8r = ade_read8(LCYCMODE);
	printf("\r\n7878 Ready");

	if(ade_init() < 0)
		printf("\nInit failed");
	else
		printf("\nInit Success");

	V_Status1RdWr_U32R = ade_read32(STATUS1);
	ade_write32(STATUS1, V_Status1RdWr_U32R);

	/////////////////////////////////////////////////
	nrk_init();
 
	nrk_time_set(0,0);
	nrk_create_taskset ();
	nrk_start();
	return 0;
}

ISR(INT3_vect)
{
	uint8_t i;

	EIMSK = 0x00;

	//Set MOSI and SCK as output, others as input
	DDRB|=  _BV(PORTB0) | _BV(PORTB1) | _BV(PORTB2);
	DDRB&=  ~(_BV(PORTB3));
	PORTB|= _BV(PORTB0) | _BV(PORTB1) | _BV(PORTB2);

	
	for(i=0; i<20; i++)
	{
		PORTB ^= _BV(PORTB0);
	//	nrk_led_toggle(GREEN_LED);
	}

	F_ChkIntEntry_U8R = 1;
}

void my_timer_callback()
{
	int8_t v;
	nrk_led_toggle(GREEN_LED);
	nrk_gpio_toggle(NRK_DEBUG_0);
	//	printf("#");
	

	V_Awatt_S32R	=	ade_read32(AWATT);
	V_Avar_S32R		=	ade_read32(AVAR);
	
	memcpy(&V_TxBuff_U8R[V_WrPtr_U32R],&V_Awatt_S32R,3);
	V_WrPtr_U32R+=3;

	memcpy(&V_TxBuff_U8R[V_WrPtr_U32R],&V_Avar_S32R,3);
	V_WrPtr_U32R+=3;

	if(V_WrPtr_U32R == C_CircBuffSize)
	{
		V_WrPtr_U32R = 0;
		V_WrSeqNo_U8R = 1;
	}
////////////////////////////
	V_EvnDetCntr_U32R++;
	if(V_EvnDetCntr_U32R==Fs)
	{
		V_EvnDetCntr_U32R = 0;
		V_DataNew_U32R = V_Awatt_S32R;
		V_DiffData_S32R = V_DataNew_U32R - V_DataOld_U32R;
		V_DataOld_U32R = V_DataNew_U32R;
		if((V_DiffData_S32R>C_ThresholdPwr)||(V_DiffData_S32R<(-1*C_ThresholdPwr)))
		{
			F_Occ_U8R = 0;
			F_Det_U8R = 0;
		}
		else
		{
			if(F_Occ_U8R==0)
			{
				F_Det_U8R = 1;
				F_Occ_U8R = 1;
			}
			else
				F_Det_U8R = 0;
		}
		V_RdDSP_U16R = ade_read16(RUN);
		if(V_RdDSP_U16R==0)
		{
			ade_write16(RUN, START);
			printf("\r\n$$");
		}
	}
/////////////////////////////
	if(F_ReadyForSignal_U8R==1)
	{
		if(((V_WrSeqNo_U8R*C_CircBuffSize + V_WrPtr_U32R)-(V_RdPtr_U32R)) >= C_SlipPktSize)
		{
		
			v = nrk_event_signal(signal_one);
			if(v==NRK_ERROR){}
		}
	}
	
	
	V_TINTcnt_U8R++;
	if(V_TINTcnt_U8R==250)
	{
		F_1secData_U8R = 1;
		V_TINTcnt_U8R=0;
		V_ArmsCurr_U32R = ade_read32(AIRMS);
		V_ArmsVolt_U32R = ade_read32(AVRMS);
		V_ArmsWatt_S32R = ade_read32(AWATT);
	}

	//	nrk_kprintf( PSTR("*** Timer interrupt!\r\n"));
}

void Task1()
{
	uint16_t cnt;
	uint8_t val;

	printf( "My node's address is %d\r\n",NODE_ADDR );

	printf( "Task1 PID=%d\r\n",nrk_get_pid());
	cnt=0;

  // Setup application timer with:
  //       Prescaler = 5 
  //       Compare Match = 25000
  //       Sys Clock = 7.3728 MHz
  // Prescaler 5 means divide sys clock by 1024
  // 7372800 / 1024 = 7200 Hz clock
  // 1 / 7200 = 0.138 ms per tick
  // 0.138 ms * 25000 = ~3472 ms / per interrupt callback
  
  // Setup application timer with:
  //       Prescaler = 5 
  //       Compare Match = 8000
  //       Sys Clock = 16 MHz
  // Prescaler 2 means divide sys clock by 8
  // 16MHz / 8 = 2MHz clock
  // 1 / 2MHz = 0.5 us per tick
  // 0.5 us * 8000 = 4 ms / per interrupt callback

  //val=nrk_timer_int_configure(NRK_APP_TIMER_0, 5, 25000, &my_timer_callback );
  
//  val=nrk_timer_int_configure(NRK_APP_TIMER_0, 5, 15625, &my_timer_callback );//1sec
//  val=nrk_timer_int_configure(NRK_APP_TIMER_0, 5, 3125, &my_timer_callback );//1sec/5
//	val=nrk_timer_int_configure(NRK_APP_TIMER_0, 5, 625, &my_timer_callback );//1sec/25
  val=nrk_timer_int_configure(NRK_APP_TIMER_0, 2, 8000, &my_timer_callback );//4msec
//  val=nrk_timer_int_configure(NRK_APP_TIMER_0, 2, 2000, &my_timer_callback );//1msec
  if(val==NRK_OK) nrk_kprintf( PSTR("Callback timer setup\r\n"));
  else nrk_kprintf( PSTR("Error setting up timer callback\r\n"));

  // Zero the timer...
  nrk_timer_int_reset(NRK_APP_TIMER_0);
  // Start the timer...
  nrk_timer_int_start(NRK_APP_TIMER_0);

  while(1) {
	//	cnt=nrk_timer_int_read(NRK_APP_TIMER_0);
	//	printf( "Task1 timer=%u\r\n",cnt );
	nrk_wait_until_next_period();
	}
}

void Task2()//For stopping the timer
{
  uint8_t cnt;
  printf( "Task2 PID=%d\r\n",nrk_get_pid());
  cnt=0;
  while(1) {
//	nrk_led_toggle(GREEN_LED);
	printf( "Task2 cnt=%d\r\n",cnt );
	nrk_wait_until_next_period();
	cnt++;
  /*	if(cnt==25) 
		{
		nrk_led_clr(ORANGE_LED);
		nrk_kprintf( PSTR("*** Timer stopped by Task2!\r\n" ));
		nrk_timer_int_stop(NRK_APP_TIMER_0);
		}
*/	}
}

void Task3()
{
uint16_t cnt;
uint16_t i;
  printf( "Task3 PID=%d\r\n",nrk_get_pid());
  cnt=0;
  while(1)
  {
	//printf( "Task3 cnt=%d\r\n",cnt );
//	V_ArmsCurr_U32R = ade_read32(AIRMS);
//	V_ArmsVolt_U32R = ade_read32(AVRMS);
//	printf("\r\nV = %ld",V_ArmsVolt_U32R);
//	printf("\r\nI = %ld",V_ArmsCurr_U32R);
//	printf("\r\n#%d;",cnt);
//	printf("#");
	cnt++;
	nrk_wait_until_next_period();

	}
}


void Task4()
{
	int8_t v;
	uint8_t cnt;
	nrk_sig_mask_t my_sigs;

	cnt =0;
	printf("Node's addre is %d\r\n",NODE_ADDR);
	printf("Task4 PID=%d\r\n",nrk_get_pid());
  
  while(1)
  {
	//	nrk_led_toggle(ORANGE_LED);

		V_5Sec_U8R++;
		if(V_5Sec_U8R==5)
		{
			V_5Sec_U8R=0;
	//		v = nrk_event_signal(signal_one);
	//		if(v==NRK_ERROR)
	//			nrk_kprintf( PSTR("T1 nrk_event_signal failed\r\n"));
			cnt++;
		}
		else
		{
		}

		nrk_wait_until_next_period();
	}
}

void Task5()
{
	int8_t v;
	nrk_sig_mask_t my_sigs;
	nrk_sig_t t;
	nrk_time_t timeout;

//	timeout.secs=10;
//	timeout.nano_secs=0;
	printf( "Task5 PID=%d\r\n",nrk_get_pid());
	v=nrk_signal_register(signal_one);

//	v=nrk_signal_register(nrk_wakeup_signal);
//	if(v==NRK_ERROR)
//		nrk_kprintf( PSTR ("T2 nrk_signal_register failed\r\n"));

	while(1)
	{
	//	nrk_led_toggle(BLUE_LED);
	//	nrk_set_next_wakeup(timeout);
//		printf("ee");
	//	my_sigs=nrk_event_wait(SIG(signal_one) | SIG(nrk_wakeup_signal));
		F_ReadyForSignal_U8R=1;
		my_sigs=nrk_event_wait(SIG(signal_one));
		nrk_led_set(RED_LED);
		F_ReadyForSignal_U8R=0;

		if(my_sigs==0)
			nrk_kprintf( PSTR ("T2 nrk_event_wait failed\r\n"));
		if(my_sigs & SIG(signal_one))
		{
		//	nrk_kprintf( PSTR( "T2 got S1\r\n"));

//		nrk_kprintf( PSTR("*"));

	slip_tx(&V_TxBuff_U8R[V_RdPtr_U32R],C_SlipPktSize,&V_TxBuff_U8R[C_CircBuffSize-1],C_CircBuffSize);

//	printf("\r\n%d",V_WrPtr_U32R);
//	printf(",%d",V_RdPtr_U32R);
			V_RdPtr_U32R=V_RdPtr_U32R+C_SlipPktSize;
			if(V_RdPtr_U32R>=C_CircBuffSize)
			{
				V_RdPtr_U32R = V_RdPtr_U32R - C_CircBuffSize;
				V_WrSeqNo_U8R = 0;
			}
			
			if(F_Det_U8R==1)
			{
				printf("\r\nEvent!");
				F_Det_U8R = 0;
			}

			if(F_1secData_U8R==1)
			{
				F_1secData_U8R = 0;
				V_CalcTemp1_U32R = V_ArmsVolt_U32R*33;
				V_CalcTemp2_U32R = V_CalcTemp1_U32R/1000000;
			//	printf("\r\nV = %ld",V_ArmsVolt_U32R);
				printf("\r\n%ld V",V_CalcTemp2_U32R);

				V_CalcTemp1_U32R = V_ArmsCurr_U32R*125;
				V_CalcTemp2_U32R = V_CalcTemp1_U32R/10000;
				printf(", %ld mA",V_CalcTemp2_U32R);
			//	printf(", I = %ld",V_ArmsCurr_U32R);

				V_CalcTemp1_S32R = V_ArmsWatt_S32R;
				V_CalcTemp2_S32R = V_CalcTemp1_S32R/10;
				V_CalcTemp1_S32R = V_CalcTemp2_S32R*344;
				V_CalcTemp2_S32R = V_CalcTemp1_S32R/100000;
			//	V_AwattCalc1_U32R = V_Awatt_S32R*3;
			//	V_AwattCalc2_U32R = (V_AwattCalc1_U32R>>13);
			//	printf(" P = %ld",V_AwattCalc2_U32R);i
				printf(", P = %ld W",V_CalcTemp2_S32R);
				V_RdDSP_U16R = ade_read16(RUN);
				if(V_RdDSP_U16R==0)
				{
					ade_write16(RUN,START);
				//	printf("\nDSP restarted");
					printf("\r\r$$");
				}
			}
			nrk_led_clr(RED_LED);
		}
	//	if(my_sigs & SIG(nrk_wakeup_signal))
	//		nrk_kprintf( PSTR( "T2 timeout\r\n"));
		
	//	nrk_wait_until_next_period();
	}
}

void
nrk_create_taskset()
{
  TaskOne.task = Task1;
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

  TaskTwo.task = Task2;
  TaskTwo.Ptos = (void *) &Stack2[NRK_APP_STACKSIZE];
  TaskTwo.Pbos = (void *) &Stack2[0];
  TaskTwo.prio = 2;
  TaskTwo.FirstActivation = TRUE;
  TaskTwo.Type = BASIC_TASK;
  TaskTwo.SchType = PREEMPTIVE;
  TaskTwo.period.secs = 0;
  TaskTwo.period.nano_secs = 500*NANOS_PER_MS;
  TaskTwo.cpu_reserve.secs = 0;
  TaskTwo.cpu_reserve.nano_secs = 100*NANOS_PER_MS;
  TaskTwo.offset.secs = 0;
  TaskTwo.offset.nano_secs= 0;
  //nrk_activate_task (&TaskTwo);


  TaskThree.task = Task3;
  TaskThree.Ptos = (void *) &Stack3[NRK_APP_STACKSIZE];
  TaskThree.Pbos = (void *) &Stack3[0];
  TaskThree.prio = 3;
  TaskThree.FirstActivation = TRUE;
  TaskThree.Type = BASIC_TASK;
  TaskThree.SchType = PREEMPTIVE;
  TaskThree.period.secs = 0;
  TaskThree.period.nano_secs = 500*NANOS_PER_MS;
  TaskThree.cpu_reserve.secs = 0;
  TaskThree.cpu_reserve.nano_secs = 100*NANOS_PER_MS;
  TaskThree.offset.secs = 0;
  TaskThree.offset.nano_secs= 0;
	nrk_activate_task (&TaskThree);


  TaskFour.task = Task4;
  nrk_task_set_stk( &TaskFour, Stack4, NRK_APP_STACKSIZE);
//  TaskFour.Ptos = (void *) &Stack3[NRK_APP_STACKSIZE];
//  TaskFour.Pbos = (void *) &Stack3[0];
  TaskFour.prio = 4;
  TaskFour.FirstActivation = TRUE;
  TaskFour.Type = BASIC_TASK;
  TaskFour.SchType = PREEMPTIVE;
  TaskFour.period.secs = 0;
  TaskFour.period.nano_secs = 20*NANOS_PER_MS;
  TaskFour.cpu_reserve.secs = 0;
  TaskFour.cpu_reserve.nano_secs = 50*NANOS_PER_MS;
  TaskFour.offset.secs = 0;
  TaskFour.offset.nano_secs= 0;
  nrk_activate_task (&TaskFour);

  TaskFive.task = Task5;
//  nrk_task_set_stk( &TaskFive, Stack5, NRK_APP_STACKSIZE);
  TaskFive.Ptos = (void *) &Stack5[NRK_APP_STACKSIZE];
  TaskFive.Pbos = (void *) &Stack5[0];
  TaskFive.prio = 1;
  TaskFive.FirstActivation = TRUE;
  TaskFive.Type = BASIC_TASK;
  TaskFive.SchType = PREEMPTIVE;
  TaskFive.period.secs = 5;
  TaskFive.period.nano_secs = 100*NANOS_PER_MS;
  TaskFive.cpu_reserve.secs = 3;
  TaskFive.cpu_reserve.nano_secs = 0*NANOS_PER_MS;
  TaskFive.offset.secs = 0;
  TaskFive.offset.nano_secs= 0;
  nrk_activate_task (&TaskFive);
}
