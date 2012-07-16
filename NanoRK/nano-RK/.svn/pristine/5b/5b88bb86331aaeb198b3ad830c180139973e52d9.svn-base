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
#include <bmac.h>
#include <nrk_driver_list.h>
#include <nrk_driver.h>
#include <twi_base_calls.h>
#include "compass.h"
#include "expansion.h"
#include <hal_firefly3.h>
#include <avr/interrupt.h>
#include <nrk_pin_define.h>
#include <nrk_events.h>

#define BUFFER_SIZE 80
#define BASE_MS 200
#define MAC_ADDR        0x0002
nrk_task_type RX_TASK;
NRK_STK rx_task_stack[NRK_APP_STACKSIZE];
void rx_task (void);

nrk_task_type TX_TASK;
NRK_STK tx_task_stack[NRK_APP_STACKSIZE];
void tx_task (void);

void nrk_create_taskset ();

uint8_t tx_buf[RF_MAX_PAYLOAD_SIZE];
uint8_t rx_buf[RF_MAX_PAYLOAD_SIZE];
drk_packet temp_packet;

uint8_t packet_len=0;
uint8_t packet_ready=0;

NRK_STK Stack1[NRK_APP_STACKSIZE];
NRK_STK Stack2[NRK_APP_STACKSIZE];
nrk_task_type TaskOne;
nrk_task_type TaskTwo;

/*Compass Variables*/
int16_t acc_x, acc_y, acc_z, mag_x, mag_y, mag_z;

/*Barometer Variables*/
int8_t temperature, pressure;

/*Configuration Variables*/
int8_t barom_period, compass_period, gps_period;

char buf[BUFFER_SIZE];
//char receive_buf[BUFFER_SIZE];
int uart1_rx_signal;

void Task1(void);
void Task2(void);

void nrk_create_taskset();
void nrk_register_drivers();
uint8_t kill_stack(uint8_t val);

int read_temp_barometer(uint8_t *buffer, uint8_t size);
int read_press_barometer(uint8_t *buffer, uint8_t size);
int barometer_enable();

uint8_t nrk_uart_data_ready_gps(uint8_t uart_num);
char getc_gps();
void nrk_setup_uart_gps(uint16_t baudrate);

void process_line();

int
main ()
{
  nrk_setup_ports();
  nrk_setup_uart(UART_BAUDRATE_115K2);
  nrk_setup_uart_gps(UART_BAUDRATE_57K6);

  printf( PSTR("starting...\r\n") );

  nrk_init();
  nrk_time_set(0,0);

  bmac_task_config ();

  //Multiples of 200ms
  gps_period = 1; 
  //Multiples of 25ms
  compass_period = 1; 
  barom_period = 1;

  nrk_register_drivers();
  nrk_create_taskset ();
  nrk_start();
  
  return 0;
}



void rx_task ()
{
  uint8_t i, len, n, checksum, offset=0;
  int8_t rssi, val;
  uint8_t *local_rx_buf;
  //nrk_time_t check_period;
  printf ("rx_task PID=%d\r\n", nrk_get_pid ());

  // init bmac on channel 25 
  bmac_init (25);

  // Enable AES 128 bit encryption
  // When encryption is active, messages from plaintext
  // source will still be received. 
  //bmac_encryption_set_key(aes_key,16);
  //bmac_encryption_enable();


  while (1) {
  bmac_rx_pkt_set_buffer (rx_buf, RF_MAX_PAYLOAD_SIZE);
    // Wait until an RX packet is received
    val = bmac_wait_until_rx_pkt ();
    // Get the RX packet 
    local_rx_buf = bmac_rx_pkt_get (&len, &rssi);
	
   
  //n = snprintf(buf, BUFFER_SIZE, "$RAD_PACKET:");
  printf("$RAD_PACKET:");
	for(i=0;i<len-2;i++)
	  printf("%c",local_rx_buf[i]);
	printf("\r\n");
    // Release the RX buffer so future packets can arrive 
  bmac_rx_pkt_release ();
  }

}


void tx_task ()
{
  uint8_t val, cnt=0;
//  bmac_addr_decode_dest_mac(0x1215);
  printf ("tx_task PID=%d\r\n", nrk_get_pid ());
  	bmac_addr_decode_set_my_mac(MAC_ADDR);
  	bmac_addr_decode_enable();
	bmac_auto_ack_enable();

  // Wait until the tx_task starts up bmac
  // This should be called by all tasks using bmac that
  // do not call bmac_init()...
  while (!bmac_started ())
    nrk_wait_until_next_period ();

  while (1) {

    if(packet_ready) {
  	  bmac_addr_decode_dest_mac(0x0002);
      val=bmac_tx_pkt(tx_buf, packet_len);
      if(val==NRK_OK) cnt++;
      else nrk_kprintf( PSTR( "NO ack or Reserve Violated!\r\n" ));
	  packet_ready=0;
      // Task gets control again after TX complete
      nrk_kprintf (PSTR ("Tx task sent data!\r\n"));
	}
    nrk_wait_until_next_period ();
  }

}



void Task1()
{
  char c;
  int i=0;
  nrk_sig_t uart0_rx_signal;

  uart0_rx_signal=nrk_uart_rx_signal_get();
  nrk_signal_register(uart0_rx_signal);


  while(1) {
  	if(nrk_uart_data_ready(NRK_DEFAULT_UART)) {
		c=getchar();
		if(c=='$' || i > 0) {
			buf[i++]=c;
		}
		if(c=='\n') {
			buf[i]='\0';
			packet_len = i;
			process_line();
			i=0;
		}
	}
    else{
	  nrk_event_wait(SIG(uart0_rx_signal));
	}
  }
}

void process_line() {
	int offset=0;
	int i, checksum=0;
	if(strncmp(buf,"$RAD_PACKET:",12)==0) {
	  	packet_len-=12;
		memcpy(&tx_buf, &(buf[12]),packet_len);
	    packet_ready=1;
	}
}

void Task2()
{
  int i,n,checksum,index=0;
  uint32_t bbuf;
  uint32_t temperature, pressure;
  int barom_update, compass_update;
  char c;
  int line_cnt=0;
  //nrk_time_t cur_time,end_time,diff_time;

  printf( "My node's address is %d\r\n",NODE_ADDR );

  printf( "Task2 PID=%d\r\n",nrk_get_pid());

  acc_x=0;
  acc_y=0;
  acc_z=0;
  mag_x=0;
  mag_y=0;
  mag_z=0;
  barom_update=1;
  compass_update=1;

  compass_enable();
  barometer_enable();

  /*Send Barometer Calibration Data*/
  n = snprintf(buf, BUFFER_SIZE, "$BMPCAL,%hd,%hd,%hd,%hu,%hu,%hu,%hd,%hd,%hd,%hd,%hd*",
                                AC1,AC2,AC3,AC4,AC5,AC6, B1, B2, MB, MC, MD);
  checksum = 0;
  for(i = 1; i < n - 1; i++)
  {
	checksum ^= buf[i];
  }
  snprintf(buf + n, BUFFER_SIZE - n, "%02X\r\n", checksum);
  printf(buf);

  while(1) {
	//nrk_time_get(&cur_time);
	
	if(gps_period != 0 && nrk_uart_data_ready_gps(1)) {
	  line_cnt=0; index=0;
	  while (line_cnt!=1) {
		  if(nrk_uart_data_ready_gps(1)){
		  	c = getc_gps();
	      	buf[index++]=c;
			if(c=='\n')
			  line_cnt++;
		  }
	    }
	  buf[index]='\0';
	  printf(buf);
	}
	else{


	/*Update Barometer Data*/
	barom_update--;
	if(barom_update==0) {
	  read_temp_barometer(&bbuf,4);
	  temperature = bbuf;
	  read_press_barometer(&bbuf,4);
	  pressure = bbuf;

	  n = snprintf(buf, BUFFER_SIZE, "$BMP085,%lu,%lu*", temperature, pressure);
	  checksum = 0;
      for(i = 1; i < n - 1; i++)
      {
	    checksum ^= buf[i];
      }
      snprintf(buf + n, BUFFER_SIZE - n, "%02X\r\n", checksum);
      printf(buf);
	  barom_update=barom_period;
    }

	compass_update--;
	if(compass_update==0) {
	  /*Update Compass Data*/
	  compass_read();
	  //printf("acc_x=%d acc_y=%d acc_z=%d\r\n",acc_x,acc_y,acc_z);
	  //printf("mag_x=%d mag_y=%d mag_z=%d\r\n",mag_x,mag_y,mag_z);

	  n = snprintf(buf, BUFFER_SIZE, "$COMPASS,0,0,0,%hd,%hd,%hd*",mag_x,mag_y,mag_z);
      checksum = 0;
      for(i = 1; i < n - 1; i++)
      {
	    checksum ^= buf[i];
      }
      snprintf(buf + n, BUFFER_SIZE - n, "%02X\r\n", checksum);
      printf(buf);
	  compass_update=compass_period;
	}
	//nrk_time_get(&end_time);
	//nrk_time_sub(&diff_time, end_time, cur_time);
	//printf("Start:%lu.%lu\r\n",diff_time.secs,diff_time.nano_secs);
	nrk_wait_until_next_period();
   }
  }

}


void
nrk_create_taskset()
{
  TaskOne.task = Task1;
  nrk_task_set_stk( &TaskOne, Stack1, NRK_APP_STACKSIZE);
  TaskOne.prio = 2;
  TaskOne.FirstActivation = TRUE;
  TaskOne.Type = BASIC_TASK;
  TaskOne.SchType = PREEMPTIVE;
  TaskOne.period.secs = 0;
  TaskOne.period.nano_secs = 50*NANOS_PER_MS; //*NANOS_PER_MS;
  TaskOne.cpu_reserve.secs = 0;
  TaskOne.cpu_reserve.nano_secs = 0*NANOS_PER_MS;
  TaskOne.offset.secs = 0;
  TaskOne.offset.nano_secs= 0;
  nrk_activate_task (&TaskOne);

  TaskTwo.task = Task2;
  nrk_task_set_stk( &TaskTwo, Stack2, NRK_APP_STACKSIZE);
  TaskTwo.prio = 1;
  TaskTwo.FirstActivation = TRUE;
  TaskTwo.Type = BASIC_TASK;
  TaskTwo.SchType = NONPREEMPTIVE;
  TaskTwo.period.secs = 0;
  TaskTwo.period.nano_secs = 20*NANOS_PER_MS; //*NANOS_PER_MS;
  TaskTwo.cpu_reserve.secs = 0;
  TaskTwo.cpu_reserve.nano_secs =  0;//200*NANOS_PER_MS;
  TaskTwo.offset.secs = 0;
  TaskTwo.offset.nano_secs= 0;
  nrk_activate_task (&TaskTwo);

  RX_TASK.task = rx_task;
  nrk_task_set_stk( &RX_TASK, rx_task_stack, NRK_APP_STACKSIZE);
  RX_TASK.prio = 2;
  RX_TASK.FirstActivation = TRUE;
  RX_TASK.Type = BASIC_TASK;
  RX_TASK.SchType = PREEMPTIVE;
  RX_TASK.period.secs = 1;
  RX_TASK.period.nano_secs = 0;
  RX_TASK.cpu_reserve.secs = 1;
  RX_TASK.cpu_reserve.nano_secs = 500 * NANOS_PER_MS;
  RX_TASK.offset.secs = 0;
  RX_TASK.offset.nano_secs = 0;
  nrk_activate_task (&RX_TASK);

  TX_TASK.task = tx_task;
  nrk_task_set_stk( &TX_TASK, tx_task_stack, NRK_APP_STACKSIZE);
  TX_TASK.prio = 2;
  TX_TASK.FirstActivation = TRUE;
  TX_TASK.Type = BASIC_TASK;
  TX_TASK.SchType = PREEMPTIVE;
  TX_TASK.period.secs = 1;
  TX_TASK.period.nano_secs = 0;
  TX_TASK.cpu_reserve.secs = 1;
  TX_TASK.cpu_reserve.nano_secs = 500 * NANOS_PER_MS;
  TX_TASK.offset.secs = 0;
  TX_TASK.offset.nano_secs = 0;
  nrk_activate_task (&TX_TASK);

}

void nrk_register_drivers()
{
}


int compass_enable(){
  	uint8_t write_buf[3];

	init_i2c();
	set_i2c_device(ACC_ADDRESS);

	write_buf[0] = 0x27;
	/* 0x27 = 0b00100111
	 * Normal power mode, all axes enabled */
	if (ee24xx_write_bytes(CTRL_REG1_A, 1, write_buf) < 0)
	  printf("Error writing acc mode\r\n");

	set_i2c_device(MAG_ADDRESS);
	write_buf[0] = 0x18;
	write_buf[1] = 0x60;
	write_buf[2] = 0x00;

	if (ee24xx_write_bytes(CRA_REG_M, 3, write_buf) < 0)
	  printf("Error writing mag mode\r\n");

	return 1;
}

int compass_read(){
	uint8_t rx_buf[6];
	int ret;

	set_i2c_device(ACC_ADDRESS);

	if ((ret = ee24xx_read_bytes(OUT_X_L_A | (1 << 7), 6, rx_buf)) < 6) {
		printf("Error reading 6 bytes from accelerometer: %d\r\n",ret);
		return -1;
	}

	uint8_t xla = rx_buf[0];
	uint8_t xha = rx_buf[1];
	uint8_t yla = rx_buf[2];
	uint8_t yha = rx_buf[3];
	uint8_t zla = rx_buf[4];
	uint8_t zha = rx_buf[5];

	acc_x = (xha << 8 | xla) >> 4;
	acc_y = (yha << 8 | yla) >> 4;
	acc_z = (zha << 8 | zla) >> 4;

	
	set_i2c_device(MAG_ADDRESS);

	if (ee24xx_read_bytes(OUT_X_H_M | (1<<7), 6, rx_buf) < 6) {
		printf("Error reading 6 bytes from magnometer");
		return -1;
	}

	uint8_t xhm = rx_buf[0];
	uint8_t xlm = rx_buf[1];
	uint8_t zhm = rx_buf[2];
	uint8_t zlm = rx_buf[3];
	uint8_t yhm = rx_buf[4];
	uint8_t ylm = rx_buf[5];

	mag_x = (int)(xhm << 8) | xlm;
	mag_y = (int)(yhm << 8) | ylm;
	mag_z = (int)(zhm << 8) | zlm;
	//mag_x = xhm;
	//mag_y = yhm;
	//mag_z = zhm;

	return 1;
}

int barometer_enable(){
	init_i2c();
	set_i2c_device(BOSCH_EEPROM_ADDRESS);
	get_eeprom_values();

	return 1;
}

int read_temp_barometer(uint8_t *buffer, uint8_t size) {
	int32_t value_from_sensor;
	uint8_t count = 0;

	if(size != 4)
	  return 0;

	set_i2c_device(BOSCH_EEPROM_ADDRESS);
	
	value_from_sensor = calc_temp();

	buffer[count] = value_from_sensor & 0xFF;
	count++;
	buffer[count] = (value_from_sensor >> 8 ) & 0xFF;
	count++;
	buffer[count] = (value_from_sensor >> 16 ) & 0xFF;
	count++;
	buffer[count] = (value_from_sensor >> 24 ) & 0xFF;

	count++;
	return count;
}

int read_press_barometer(uint8_t *buffer, uint8_t size){
	int32_t value_from_sensor;
	uint8_t count = 0;

	if(size != 4)
	  return 0;

	set_i2c_device(BOSCH_EEPROM_ADDRESS);
	
	calc_temp();
	value_from_sensor = calc_press(0);

	buffer[count] = value_from_sensor & 0xFF;
	count++;
	buffer[count] = (value_from_sensor >> 8 ) & 0xFF;
	count++;
	buffer[count] = (value_from_sensor >> 16 ) & 0xFF;
	count++;
	buffer[count] = (value_from_sensor >> 24 ) & 0xFF;

	count++;
	return count;
}

int32_t calc_true_press(uint8_t oss){

    B6 = B5 - 4000;                                     if(DEBUG) printf("B6 %li\r\n", B6);
//    X1 = (B2 * (B6 * B6/ pow(2,12)))/ pow(2,11);        if(DEBUG) printf("X1 %li\r\n", X1);
    X1 = (B2 * (B6 * B6/ 4096))/ 2048;        if(DEBUG) printf("X1 %li\r\n", X1);
    X2 = (int32_t)AC2 * B6 / 2048;                          if(DEBUG) printf("X2 %li\r\n", X2);
//    X2 = AC2 * B6 / pow(2,11);                          if(DEBUG) printf("X2 %li\r\n", X2);
    X3 = X1 + X2;                                       if(DEBUG) printf("X3 %li\r\n", X3);
    B3 = ((((int32_t)AC1 * 4 + X3) << oss) + 2) / 4;               if(DEBUG) printf("B3 %li\r\n", B3);
//    X1 = AC3 * B6 / pow(2,13);                          if(DEBUG) printf("X1 %li\r\n", X1);
    X1 = AC3 * B6 / 8192;                          if(DEBUG) printf("X1 %li\r\n", X1);
//    X2 = (B1 * (B6 * B6 / pow(2,12))) / pow(2,16);      if(DEBUG) printf("X2 %li\r\n", X2);
    X2 = (B1 * (B6 * B6 / 4096)) / 65536;      if(DEBUG) printf("X2 %li\r\n", X2);
//    X3 = ((X1 + X2) + 2) / pow(2,2);                    if(DEBUG) printf("X3 %li\r\n", X3);
    X3 = ((X1 + X2) + 2) / 4;                    if(DEBUG) printf("X3 %li\r\n", X3);
//    B4 = AC4 * (X3 + 32768) / pow(2,15);                if(DEBUG) printf("B4 %lu\r\n", B4);
    B4 = AC4 * (X3 + 32768) / 32768;                if(DEBUG) printf("B4 %lu\r\n", B4);
    B7 = (UP - B3) * (50000 >> oss);                    if(DEBUG) printf("B7 %lu\r\n", B7);

    if(B7 < 0x80000000)
       P = (B7 * 2)/ B4;
    else
        P = (B7 / B4) * 2;
                                                        if(DEBUG) printf("P %li\r\n", P);
    X1 = (P / 256) * (P / 256);               if(DEBUG) printf("X1 %li\r\n", X1);
//    X1 = (P / pow(2,8)) * (P / pow(2,8));               if(DEBUG) printf("X1 %li\r\n", X1);
    X1 = (X1 * 3038) / 65536;                       if(DEBUG) printf("X1 %li\r\n", X1);
//    X1 = (X1 * 3038) / pow(2,16);                       if(DEBUG) printf("X1 %li\r\n", X1);
    X2 = (-7357 * P ) / 65536;                      if(DEBUG) printf("X2 %li\r\n", X2);
//	X2 = (-7357 * P ) / pow(2,16);                      if(DEBUG) printf("X2 %li\r\n", X2);
//    P = P + (X1 + X2 + 3791) / pow(2,4);                if(DEBUG) printf("P %li\r\n", P);
    P = P + (X1 + X2 + 3791) / 16;                if(DEBUG) printf("P %li\r\n", P);

    if(DEBUG) printf("Pressure is %li Pascals\r\n", P);

    return P;

}


inline int32_t calc_true_temp(){

    X1 = ((int32_t)UT - (int32_t)AC6) * (int32_t)AC5 / (int32_t)32768;
//    X1 = ((int32_t)UT - (int32_t)AC6) * (int32_t)AC5 / (int32_t)pow(2,15);
if(DEBUG) printf("X1 %li\r\n", X1);
//    X2 = (int32_t) MC * pow(2,11) / (X1 + MD);
    X2 = (int32_t) MC * 2048 / (X1 + MD);
if(DEBUG) printf("X2 %li\r\n", X2);
    B5 = X1 + X2;
if(DEBUG) printf("B5 %li\r\n", B5);
//    T = (B5 + 8) / pow(2,4);
    T = (B5 + 8) / 16;
if(DEBUG) printf("XT %li\r\n", T);

    if(DEBUG) printf("Temperature is %li /10 degrees Celcius.\r\n", T);

    return T;
}

void read_uncomp_temp(){

    uint8_t write_buf[1];
    uint8_t rx_buf[2];
    uint8_t i;

    write_buf[0] = 0x2E;

    // Step 1: Write 0x2E into register 0xF4
    ee24xx_write_bytes(0xF4, 1, write_buf);

    // Step 2: Wait 4.5 ms
 //   nrk_wait_ticks(1000); // 1 tick is 1 ms I believe
    nrk_spin_wait_us(4500);

    // Step 3: Read registers 0xF6, 0xF7
    ee24xx_read_bytes(0xF6, 2, rx_buf);

    if(DEBUG){
        for(i = 0; i < 2; i++)
            printf("uncomp_temp[%i] is %u\r\n", i, rx_buf[i]);
    }

    UT = (((uint16_t)rx_buf[0] | 0x0000) << 8) | ((uint16_t)rx_buf[1]);

    if(DEBUG)
     printf("UT is %i\r\n", UT);

}


int32_t calc_temp(){
    read_uncomp_temp();
    return calc_true_temp();
}


int32_t calc_press(uint8_t oss){
    read_uncomp_press(oss);
    return calc_true_press(oss);
}

void read_uncomp_press(uint8_t oss){

    uint8_t write_buf[1]; // Used to send a value to the Bosch sensor
    uint8_t rx_buf[3]; // Used to store the received values from the sensor
    uint8_t i;

    write_buf[0] = 0x34 + (oss << 6);

    // Step 1: Write 0x2E into register 0xF4
    ee24xx_write_bytes(0xF4, sizeof(write_buf), write_buf);

    // Step 2: Wait 4.5 ms
//    nrk_wait_ticks(1000); // 1 tick is 1 ms I believe
    nrk_spin_wait_us(4500);

    // Step 3: Read registers 0xF6, 0xF7, 0xF8
    ee24xx_read_bytes(PRESS_BASE_REGISTER, sizeof(rx_buf), rx_buf);

    if(DEBUG){
        for(i = 0; i < 3; i++)
            printf("uncomp_press[%i] is %u\r\n", i, rx_buf[i]);
    }

     // Assemble the raw pressure value

     UP = ((uint32_t) rx_buf[0]) << 16;
     UP = UP  | ((uint32_t)(rx_buf[1]) << 8);
     UP = UP  + rx_buf[2];
     UP = UP >> (8-oss);

     if(DEBUG)
     printf("UP is %li\r\n", UP);

}


void get_eeprom_values(){

   uint8_t raw_eeprom_data[22]; // Buffer for storing the raw values being returned
   int j = 0;
   int rv;

   // Get the values
  rv = ee24xx_read_bytes(0xAA, 22, raw_eeprom_data);

  if(DEBUG){

      printf("The raw values from the EEPROM are: \r\n");

      for(j = 0; j<22; j++)
        printf("%02x ", raw_eeprom_data[j]);
      printf("\r\n");
  }

  // Place the data in a formatted buffer
  for(j = 0; j < 11; j++){
      eeprom_values[j] = ((raw_eeprom_data[2*j] | 0x0000) << 8) |
(raw_eeprom_data[2*j + 1]);
  }

 // Print out the values of the EEPROM
  if(DEBUG){
      printf("AC1: %i\r\n", AC1);
      printf("AC2: %i\r\n", AC2);
      printf("AC3: %i\r\n", AC3);
      printf("AC4: %u\r\n", AC4);
      printf("AC5: %u\r\n", AC5);
      printf("AC6: %u\r\n", AC6);
      printf("B1: %i\r\n", B1);
      printf("B2: %i\r\n", B2);
      printf("MB: %i\r\n", MB);
      printf("MC: %i\r\n", MC);
      printf("MD: %i\r\n", MD);
      printf("-------------------------------------------------\r\n");
  }
}

static uint16_t uart1_rx_buf_start,uart1_rx_buf_end;
static char uart1_rx_buf[128];//[MAX_RX_UART_BUF];

void nrk_setup_uart_gps(uint16_t baudrate)
{
  setup_uart1(baudrate);
 
  #ifdef NRK_UART1_BUF
  uart1_rx_signal=0;
  uart1_rx_buf_start=0;
  uart1_rx_buf_end=0;
  ENABLE_UART1_RX_INT();
  #endif
}

SIGNAL(USART1_RX_vect)
{
char c;
// cli();
DISABLE_UART1_RX_INT();
   UART1_WAIT_AND_RECEIVE(c);
   uart1_rx_buf[uart1_rx_buf_end]=c;
   //if(uart_rx_buf_end==uart_rx_buf_start) sig=1; else sig=0;
   uart1_rx_buf_end++;
   //if(uart_rx_buf_end==uart_rx_buf_start) nrk_kprintf(PSTR("Buf overflow!\r\n" ));
   if(uart1_rx_buf_end==128) {
	   uart1_rx_buf_end=0;
		   }
   uart1_rx_signal=1;
CLEAR_UART1_RX_INT();
ENABLE_UART1_RX_INT();
// sei();
}

char getc_gps()
{
  char tmp;

   //if(uart1_rx_signal <= 0) nrk_kprintf(PSTR("uart1 rx sig failed\r\n" ));
     tmp=uart1_rx_buf[uart1_rx_buf_start];
     uart1_rx_buf_start++;
   if(uart1_rx_buf_start>=128) { uart1_rx_buf_start=0; }
   uart1_rx_signal=0;

   return tmp;
}


uint8_t nrk_uart_data_ready_gps(uint8_t uart_num)
{

  if(uart_num==0) {
	if( UCSR0A & BM(RXC0) ) return 1;
  }   
  if(uart_num==1) {
	if(uart1_rx_buf_start != uart1_rx_buf_end) return 1;
	//if(uart1_rx_signal > 0) return 1;
  }

  return 0;
}

