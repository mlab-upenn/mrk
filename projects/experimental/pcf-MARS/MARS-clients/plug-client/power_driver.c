#include <nrk.h>
#include <include.h>
#include <ulib.h>
#include <stdio.h>
#include <avr/sleep.h>
#include <hal.h>
#include <nrk_error.h>
#include <nrk_timer.h>
#include <power_driver.h>

//#define CT_METER
#define ADC_SETUP_DELAY  100
// Voltage divider value for 3.3K with 1.8K from 5 volts to 3.3 volts x 1000

//#define socket_0_enable()       nrk_gpio_set(NRK_DEBUG_0)
//#define socket_0_disable()      nrk_gpio_clr(NRK_DEBUG_0)
//#define socket_1_enable()       nrk_gpio_set(NRK_DEBUG_1)
//#define socket_1_disable()      nrk_gpio_clr(NRK_DEBUG_1)

//#define power_mon_enable()       nrk_gpio_clr(NRK_DEBUG_2)
//#define power_mon_disable()      nrk_gpio_set(NRK_DEBUG_2)


// ADC hardware init parameters
#define ADC_INIT() \
    do { \
        ADCSRA = BM(ADPS0) | BM(ADPS1); \
        ADMUX = BM(REFS0); \
} while (0)

#define ADC_SET_CHANNEL(channel) do { ADMUX = (ADMUX & ~0x1F) | (channel); } while (0)

// Enables/disables the ADC
#define ADC_ENABLE() do { ADCSRA |= BM(ADEN); } while (0)
#define ADC_DISABLE() do { ADCSRA &= ~BM(ADEN); } while (0)

#define ADC_VREF_VCC() \
   do { \
	ADMUX &= ~(BM(REFS1));  \
	ADMUX |= BM(REFS0);  \
} while(0)


#define ADC_VREF_1_1() \
   do { \
	ADMUX &= ~(BM(REFS0));  \
	ADMUX |= BM(REFS1);  \
} while(0)



#define ADC_VREF_2_56() \
   do { \
        ADMUX |= BM(REFS1) | BM(REFS0);  \
} while(0)

#define ADC_SAMPLE_SINGLE() \
    do { \
ADCSRA |= BM(ADSC); \
while (!(ADCSRA & 0x10)); \
} while(0)

// Macros for obtaining the latest sample value
#define ADC_GET_SAMPLE_10(x) \
do { \
x =  ADCL; \
x |= ADCH << 8; \
} while (0)

#define ADC_GET_SAMPLE_8(x) \
do { \
x = ((uint8_t) ADCL) >> 2; \
x |= ((int8_t) ADCH) << 6; \
} while (0)


uint8_t channel;

static uint32_t true_power_last, true_power_last2;

static nrk_sig_mask_t my_sigs;


int8_t power_socket_enable (uint8_t socket)
{

  if (socket == 0) {
    socket_0_enable ();
    socket_0_active = 1;
    nrk_int_disable();
    nrk_eeprom_write_byte(EEPROM_STATE_ADDR, socket_0_active);
    nrk_int_enable();
  }
  nrk_timer_int_start (NRK_APP_TIMER_0);
  return 1;
}

int8_t power_socket_disable (uint8_t socket)
{

  if (socket == 0) {
    socket_0_disable ();
    socket_0_active = 0;
    nrk_int_disable();
    nrk_eeprom_write_byte(EEPROM_STATE_ADDR, socket_0_active);
    nrk_int_enable();
  }
  /*
    if (socket_1_active == 0 && socket_0_active == 0) {
    // Turn interrupt off to save power
    nrk_timer_int_stop (NRK_APP_TIMER_0);
    // Shutdown monitor circuit to save power
    power_mon_disable ();
  }
  */
  return 1;
}

void calc_power()
{
    // FIXME: Disable interrupt later if power off to save energy
//    if(socket_0_active==0 && socket_1_active==0) return;
 //  nrk_int_disable();

    ADC_SET_CHANNEL (VOLTAGE_CHAN);
    nrk_spin_wait_us(ADC_SETUP_DELAY);
    ADC_SAMPLE_SINGLE();
    ADC_GET_SAMPLE_10(v);

    ADC_SET_CHANNEL (CURRENT_LOW);
    nrk_spin_wait_us(ADC_SETUP_DELAY);
    ADC_SAMPLE_SINGLE();
    ADC_GET_SAMPLE_10(c1);

    ADC_SET_CHANNEL (CURRENT_HIGH);
    nrk_spin_wait_us(ADC_SETUP_DELAY);
    ADC_SAMPLE_SINGLE();
    ADC_GET_SAMPLE_10(c2);

    //ADC_SET_CHANNEL (5);
    //nrk_spin_wait_us(ADC_SETUP_DELAY);
    //ADC_SAMPLE_SINGLE();
    //ADC_GET_SAMPLE_10(center_chan);
    //c1_center=(uint16_t)center_chan;

// Catch the rising edge after the voltage dips below a low threshold
// Then ignore that case until the voltage goes up again.
// This filter is used to detect the zero crossing point
//if(triggered==0 && v<VOLTAGE_LOW_THRESHOLD && v>v_last)
//if(v>VOLTAGE_ZERO_THRESHOLD && v_last<=VOLTAGE_ZERO_THRESHOLD)
if(v>v_center && v_last<=v_center)
{
//if(cycle_state==CYCLE_HIGH) cycle_state=CYCLE_LOW;
//else { 
	// Low to High transition
	cycle_state=CYCLE_HIGH; 
	cycle_cnt++; 
	if(energy_cycle<0) energy_cycle*=-1;
	energy_total+=energy_cycle;
	if(energy_cycle2<0) energy_cycle2*=-1;
	energy_total2+=energy_cycle2;
	energy_cycle=0;
	energy_cycle2=0;
//	}
cycle_started=1;
//triggered=1;
}

// Reset filter trap after the voltage goes up
//if(v>VOLTAGE_ZERO_THRESHOLD) triggered=0;

v_last=v;

    if(cycle_started==1) 
	{
	ticks++;
	if(c1<c_p2p_low) c_p2p_low=c1;
	if(c1>c_p2p_high) c_p2p_high=c1;
	if(v<v_p2p_low) v_p2p_low=v;
	if(v>v_p2p_high) v_p2p_high=v;
	c1-=c_center;

	if(c2<c_p2p_low2) c_p2p_low2=c2;
	if(c2>c_p2p_high2) c_p2p_high2=c2;
	c2-=c2_center;

        // Do we need this?	
	v-=v_center;
	// remove noise at floor
	//if(c1<10) c1=0;
	
	current_total+=((int32_t)c1*(int32_t)c1);
	voltage_total+=((int32_t)v*(int32_t)v);
	//if(cycle_state==CYCLE_LOW) v*=-1;
	energy_cycle+=((int32_t)c1*(int32_t)v);
	
	current_total2+=((int32_t)c2*(int32_t)c2);
	energy_cycle2+=((int32_t)c2*(int32_t)v);
	//printf( "c1=%u current=%lu\r\n",c1, current_total );

	
	if(ticks>=2000) 
	{

	// Save values to pass to event detector functions that calculate power for
	// packets etc
	freq=cycle_cnt;
	l_v_p2p_high=v_p2p_high;
  	l_v_p2p_low=v_p2p_low;
  	l_c_p2p_high=c_p2p_high;
  	l_c_p2p_low=c_p2p_low;
  	l_c_p2p_high2=c_p2p_high2;
  	l_c_p2p_low2=c_p2p_low2;
	ticks_last=ticks;
	current_total_last=current_total;
	current_total2_last=current_total2;
	energy_total_last=energy_total;
	energy_total2_last=energy_total2;
	voltage_total_last=voltage_total;


	// Reset values for next cycle	
  	ticks=0;
  	cycle_cnt=0;
  	voltage_total=0;
  	energy_total=0;
  	energy_total2=0;
  	current_total=0;
  	current_total2=0;
  	v_p2p_low=2000;
  	v_p2p_high=0;
  	c_p2p_low=2000;
  	c_p2p_high=0;
  	c_p2p_low2=2000;
  	c_p2p_high2=0;

	// Signal event detector task
	nrk_event_signal(update_energy_sig);
	}


	}

//nrk_int_enable();
}


void power_init ()
{
  ADC_INIT ();
  ADC_ENABLE ();
  ADC_VREF_VCC();


  // Setup the timer to fire every 1ms
  // nrk_timer_int_configure( NRK_APP_TIMER_0, 1, 16000, calc_power);
  // Setup the timer to fire every 0.5 ms
   nrk_timer_int_configure( NRK_APP_TIMER_0, 1, 8000, calc_power);
  ticks=0;
  cycle_state=CYCLE_HIGH;
  //cycle_state_last=CYCLE_UNKNOWN;
  cycle_cnt=0;
  cycle_avg=0;
  cycle_started=0;
  c_center=512;
  c2_center=512;
  v_p2p_low=2000;
  v_p2p_high=0;
  c_p2p_low=2000;
  c_p2p_high=0;
  c_p2p_low2=2000;
  c_p2p_high2=0;
  rms_current=0;
  rms_current2=0;
  rms_voltage=0;
  energy_total2=0;
  energy_total=0;
  energy_cycle2=0;
  energy_cycle=0;
//  cummulative_energy2=0;
//  cummulative_energy=0;
  total_secs=0;
  v_last=VOLTAGE_LOW_THRESHOLD+10;
  triggered=0;
  nrk_timer_int_start(NRK_APP_TIMER_0);

//  nrk_gpio_direction(NRK_DEBUG_2, NRK_PIN_OUTPUT);

  //startup_sock_state=nrk_eeprom_read_byte(0x100);
  //if((startup_sock_state&0x01)==0x01)
  //      {
                //power_mon_enable();
  //      }
  //else {
  //socket_0_active=0;
  //power_mon_disable();
  //              nrk_kprintf( PSTR("Socket inactive\r\n"));
  //}
  //if((startup_sock_state&0x02)==0x02)
  //      {
  //      }
  // else {
  // socket_1_active=0;
  // }



}



void event_detector_task()
{

update_energy_sig=nrk_signal_create();
if(update_energy_sig==NRK_ERROR)
	nrk_kprintf(PSTR("Error creating update_energy_sig signal!\r\n"));

nrk_signal_register(update_energy_sig);

if(v==NRK_ERROR) nrk_kprintf( PSTR( "nrk_signal_register failed\r\n" ));
//ev_timeout.secs=10;
//ev_timeout.nano_secs=0;
while(1)
{
// Wait on signal
//nrk_set_next_wakeup(ev_timeout);
my_sigs=nrk_event_wait( SIG(update_energy_sig) /*| SIG(nrk_wakeup_signal)*/ );
if(my_sigs!=0 && ticks_last>900)
{

  tmp_d=current_total_last / ticks_last;
  if(tmp_d<0) tmp_d=0;
  tmp_d=sqrt(tmp_d);
  rms_current=(uint16_t)tmp_d;	

  tmp_d=current_total2_last / ticks_last;
  if(tmp_d<0) tmp_d=0;
  tmp_d=sqrt(tmp_d);
  rms_current2=(uint16_t)tmp_d;	

  tmp_d=voltage_total_last / ticks_last;
  if(tmp_d<0) tmp_d=0;
  tmp_d=sqrt(tmp_d);
  rms_voltage=(uint16_t)tmp_d;	
		
  if(energy_total_last<0) energy_total_last=0;	
  true_power=energy_total_last / ticks_last;
  if(true_power>TRUE_POWER_ON_THRESH) cummulative_energy.total+=true_power;
			
  if(energy_total2_last<0) energy_total2_last=0;	
  true_power2=energy_total2_last / ticks_last;
  if(true_power2>TRUE_POWER_ON_THRESH) cummulative_energy2.total+=true_power2;
			
  total_secs++;

  // Divide by seconds per hour to give Watts per hour
  // If this is too slow, make a power of 2 shit instead...
  tmp_energy.total=cummulative_energy.total/3600;
  tmp_energy2.total=cummulative_energy2.total/3600;
  // Only care about lower 6 bytes of 8 byte long long
  // At max power this will last 239 years
  // Divide by P-scaler to get Watt*hrs

  if(total_secs%400==0 )
	{
		nrk_int_disable();
		nrk_eeprom_write_byte(EEPROM_ENERGY1_0_ADDR, cummulative_energy.byte[0]);
		nrk_eeprom_write_byte(EEPROM_ENERGY1_1_ADDR, cummulative_energy.byte[1]);
		nrk_eeprom_write_byte(EEPROM_ENERGY1_2_ADDR, cummulative_energy.byte[2]);
		nrk_eeprom_write_byte(EEPROM_ENERGY1_3_ADDR, cummulative_energy.byte[3]);
		nrk_eeprom_write_byte(EEPROM_ENERGY1_4_ADDR, cummulative_energy.byte[4]);
		nrk_eeprom_write_byte(EEPROM_ENERGY1_5_ADDR, cummulative_energy.byte[5]);
		nrk_eeprom_write_byte(EEPROM_ENERGY1_6_ADDR, cummulative_energy.byte[6]);
		nrk_eeprom_write_byte(EEPROM_ENERGY1_7_ADDR, cummulative_energy.byte[7]);
		
		nrk_eeprom_write_byte(EEPROM_ENERGY2_0_ADDR, cummulative_energy2.byte[0]);
		nrk_eeprom_write_byte(EEPROM_ENERGY2_1_ADDR, cummulative_energy2.byte[1]);
		nrk_eeprom_write_byte(EEPROM_ENERGY2_2_ADDR, cummulative_energy2.byte[2]);
		nrk_eeprom_write_byte(EEPROM_ENERGY2_3_ADDR, cummulative_energy2.byte[3]);
		nrk_eeprom_write_byte(EEPROM_ENERGY2_4_ADDR, cummulative_energy2.byte[4]);
		nrk_eeprom_write_byte(EEPROM_ENERGY2_5_ADDR, cummulative_energy2.byte[5]);
		nrk_eeprom_write_byte(EEPROM_ENERGY2_6_ADDR, cummulative_energy2.byte[6]);
		nrk_eeprom_write_byte(EEPROM_ENERGY2_7_ADDR, cummulative_energy2.byte[7]);
		nrk_int_enable();
	}

/*
// EVENT DETECTOR 

  tran_pkt.msgs_payload=async_buf;
  tran_pkt.num_msgs=0;


  //if(rms_current>current_last) delta=rms_current-current_last;
  //else delta=current_last-rms_current;
  if(true_power>true_power_last) delta=true_power-true_power_last;
  else delta=true_power_last-true_power;

  if(true_power2>true_power_last2) delta2=true_power2-true_power_last2;
  else delta=true_power_last2-true_power2;
  //if(rms_current2>current_last2) delta2=rms_current2-current_last2;
  //else delta2=current_last2-rms_current2;

if((socket_0_push_enabled && (delta>socket_0_push_threshold*100)) || (socket_1_push_enabled && (delta2>socket_1_push_threshold*100)))
{
	pd_pkt.type=DEBUG_PKT;
        pd_pkt.rms_voltage=rms_voltage;
        pd_pkt.rms_current=rms_current;
        pd_pkt.rms_current2=rms_current2;
        pd_pkt.true_power=true_power;
        pd_pkt.true_power2=true_power2;
        pd_pkt.freq=freq;
	pd_pkt.socket0_state= socket_0_active;
	pd_pkt.socket1_state= socket_1_active;
	for(len=0; len<6; len++ )
        {
		pd_pkt.energy[len]=tmp_energy.byte[len];
        	pd_pkt.energy2[len]=tmp_energy2.byte[len];
        }
	pd_pkt.current_p2p_high=l_c_p2p_high;
        pd_pkt.current_p2p_low=l_c_p2p_low;
        pd_pkt.current_p2p_high2=l_c_p2p_high2;
        pd_pkt.current_p2p_low2=l_c_p2p_low2;
        pd_pkt.voltage_p2p_high=l_v_p2p_high;
        pd_pkt.voltage_p2p_low=l_v_p2p_low;
        pd_pkt.total_secs=total_secs;


	tran_pkt.num_msgs=0;
	tran_pkt.checksum=0;
	tran_pkt.msgs_payload=&(async_buf[TRANSDUCER_PKT_HEADER_SIZE]);

	tran_msg.payload=&(async_buf[TRANSDUCER_PKT_HEADER_SIZE+TRANSDUCER_MSG_HEADER_SIZE]);
	tran_msg.type=TRAN_POWER_PKT;
	tran_msg.len=ff_power_debug_pack(tran_msg.payload, &pd_pkt);
	tran_msg.mac_addr=my_mac;
	len=transducer_msg_add( &tran_pkt, &tran_msg);

	len = transducer_pkt_pack(&tran_pkt, async_buf);
	val=push_p2p_pkt( async_buf,len,TRANSDUCER_PKT, 0x0 );
}
*/

true_power_last=true_power;
true_power_last2=true_power2;


} 
// toggle RED led if freq not above 50Hz
//if(freq==0) nrk_led_toggle(RED_LED);


}



}




