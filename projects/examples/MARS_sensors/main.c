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
*  Contributing Authors (specific to this file):
*  Zane Starr
*******************************************************************************/


#include <nrk.h>
#include <include.h>
#include <ulib.h>
#include <stdio.h>
#include <avr/sleep.h>
#include <hal.h>
#include <nrk_error.h>
#include <nrk_timer.h>
#include <nrk_eeprom.h>
#include <nrk_driver_list.h>
#include <nrk_driver.h>
#include <Wire.h>
#include <HMC5883L.h>
#include <ADXL345.h>
#include <ITG3200.h>
#include <MARS_basic_sensor.h>

NRK_STK Stack1[NRK_APP_STACKSIZE];
nrk_task_type TaskOne;
void Task1(void);


void nrk_create_taskset();
void nrk_register_drivers();
uint8_t kill_stack(uint8_t val);

int16_t acbuf[3];
int16_t gyrobuf[3];
int16_t magbuf[3];
uint8_t buffer[6];
int8_t* success;
uint16_t cnt;

int main ()
{
  uint8_t t;
  nrk_setup_ports();
  nrk_setup_uart(UART_BAUDRATE_115K2);
  begin(); //start I2C bus


  printf( PSTR("starting...\r\n") );

  nrk_init();
  nrk_led_clr (0);
  nrk_led_clr (1);
  nrk_led_clr (2);
  nrk_led_clr (3);
	
  nrk_time_set(0,0);
	
  nrk_create_taskset ();
  nrk_start();
  
  return 0;
}


/*void my_timer_callback()
{
	sei();
	get_acc_values(acbuf);
	//read gyroscope
	read_gyro_xyz_one_ptr(gyrobuf);
	//read magnetometer	
	get_Magneto_Values_scaled(magbuf);	
}*/

void Task1()
{
	nrk_time_t t;
	
	nrk_kprintf( PSTR("Booting Up ADXL345. . .\r\n") );
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
	
	/* Setup application timer with:
	 *      Prescaler = 2 
	 *     Compare Match = 5000
	 *     Sys Clock = 16MHz
	 *  Prescaler 2 means divide sys clock by 8
	 * 16000000 / 8 = 2000000 Hz clock
	 * 1 / 2000000 = 0.0005 ms per tick
	 * 0.0005 ms * 5000 = 2.5 ms / per interrupt callback
	 */
	
	/*val=nrk_timer_int_configure(NRK_APP_TIMER_0,2,50000, &my_timer_callback ); // give 400Hz timer
	if(val==NRK_OK) nrk_kprintf( PSTR("Callback timer setup\r\n"));
	else nrk_kprintf( PSTR("Error setting up timer callback\r\n"));
	
	// Zero the timer...
	nrk_timer_int_reset(NRK_APP_TIMER_0);
	
	// Start the timer...
	nrk_timer_int_start(NRK_APP_TIMER_0);*/
	
    printf ("tx_task PID=%d\r\n", nrk_get_pid ());
	
	cnt = 0;
	
	while (1) {
		nrk_led_toggle(GREEN_LED);
		
		if (cnt >= 65534) cnt = 0;
		else cnt++;
		
		//read acc
		get_acc_values(acbuf);
		//read gyroscope
		read_gyro_xyz_one_ptr(gyrobuf);
		//read magnetometer	
		get_Magneto_Values_scaled(magbuf);
		
		printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%u\r\n",acbuf[0],acbuf[1],acbuf[2],
												  gyrobuf[0],gyrobuf[1],gyrobuf[2],
												  magbuf[0],magbuf[1],magbuf[2],cnt);
		//nrk_sw_wdt_update(0); 
		nrk_wait_until_next_period();
	}
	
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
  TaskOne.period.nano_secs = 5*NANOS_PER_MS; //*NANOS_PER_MS;
  TaskOne.cpu_reserve.secs = 0;
  TaskOne.cpu_reserve.nano_secs = 0;//  200*NANOS_PER_MS;
  TaskOne.offset.secs = 0;
  TaskOne.offset.nano_secs= 0;
  nrk_activate_task (&TaskOne);

}


