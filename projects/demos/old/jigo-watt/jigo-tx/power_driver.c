#include <nrk.h>
#include <include.h>
#include <ulib.h>
#include <stdio.h>
#include <avr/sleep.h>
#include <hal.h>
#include <bmac.h>
#include <nrk_error.h>
#include <nrk_timer.h>
#include <power_driver.h>

//#define CT_METER
#define ADC_SETUP_DELAY  100
// Voltage divider value for 3.3K with 1.8K from 5 volts to 3.3 volts x 1000
#define VOLTAGE_DIVIDER_SCALER 647


#define socket_0_enable()       nrk_gpio_set(NRK_DEBUG_0)
#define socket_0_disable()      nrk_gpio_clr(NRK_DEBUG_0)
#define socket_1_enable()       nrk_gpio_set(NRK_DEBUG_1)
#define socket_1_disable()      nrk_gpio_clr(NRK_DEBUG_1)

#define power_mon_enable()       nrk_gpio_clr(NRK_DEBUG_2)
#define power_mon_disable()      nrk_gpio_set(NRK_DEBUG_2)


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

int8_t power_socket_enable (uint8_t socket)
{
  if (socket != 0 && socket != 1)
    return 0;

  power_mon_enable ();
  if (socket == 0) {
    socket_0_enable ();
    socket_0_active = 1;
  }
  if (socket == 1) {
    socket_1_enable ();
    socket_1_active = 1;
  }
  nrk_timer_int_start (NRK_APP_TIMER_0);
  return 1;
}

int8_t power_socket_disable (uint8_t socket)
{
  if (socket != 0 && socket != 1)
    return 0;

  if (socket == 0) {
    socket_0_disable ();
    socket_0_active = 0;
  }
  if (socket == 1) {
    socket_1_disable ();
    socket_1_active = 0;
  }
  if (socket_1_active == 0 && socket_0_active == 0) {
    // Turn interrupt off to save power
    nrk_timer_int_stop (NRK_APP_TIMER_0);
    // Shutdown monitor circuit to save power
    power_mon_disable ();
  }
  return 1;
}

/********************************************************************************************* 
* calc_power()
*
* This function is called at 1000Hz NRK_APP_TIMER_0.  It counts ticks starting from the first
* zero crossing point and updates the user power variables once per second (when ticks==1000).
**********************************************************************************************/ 
void calc_power()
{
    // FIXME: Disable interrupt later if power off to save energy

#ifdef CT_METER
    ADC_SET_CHANNEL (1);
    nrk_spin_wait_us(ADC_SETUP_DELAY);
    ADC_SAMPLE_SINGLE();
    ADC_GET_SAMPLE_10(v);

    ADC_SET_CHANNEL (2);
    nrk_spin_wait_us(ADC_SETUP_DELAY);
    ADC_SAMPLE_SINGLE();
    ADC_GET_SAMPLE_10(c1);

    ADC_SET_CHANNEL (3);
    nrk_spin_wait_us(ADC_SETUP_DELAY);
    ADC_SAMPLE_SINGLE();
    ADC_GET_SAMPLE_10(c2);

    ADC_SET_CHANNEL (5);
    nrk_spin_wait_us(ADC_SETUP_DELAY);
    ADC_SAMPLE_SINGLE();
    ADC_GET_SAMPLE_10(center_chan);
    c1_center=(uint16_t)center_chan;

#else
    if(socket_0_active==0) return;
    ADC_SET_CHANNEL (4);
    nrk_spin_wait_us(ADC_SETUP_DELAY);
    ADC_SAMPLE_SINGLE();
    ADC_GET_SAMPLE_10(v);

    ADC_SET_CHANNEL (5);
    nrk_spin_wait_us(ADC_SETUP_DELAY);
    ADC_SAMPLE_SINGLE();
    ADC_GET_SAMPLE_10(c1);

    ADC_SET_CHANNEL (6);
    nrk_spin_wait_us(ADC_SETUP_DELAY);
    ADC_SAMPLE_SINGLE();
    ADC_GET_SAMPLE_10(c2);

    ADC_SET_CHANNEL (7);
    nrk_spin_wait_us(ADC_SETUP_DELAY);
    ADC_SAMPLE_SINGLE();
    ADC_GET_SAMPLE_10(center_chan);
    c1_center=(uint16_t)(((uint32_t)center_chan*647)/1000);
#endif

// Catch the rising edge after the voltage dips below a low threshold
// Then ignore that case until the voltage goes up again.
// This filter is used to detect the zero crossing point
if(triggered==0 && v<VOLTAGE_LOW_THRESHOLD && v>v_last)
{
if(cycle_state==CYCLE_HIGH) cycle_state=CYCLE_LOW;
else {
        // Low to High transition
        cycle_state=CYCLE_HIGH;
        cycle_cnt++;
        if(energy_cycle<0) energy_cycle*=-1;
        energy_total+=energy_cycle;
        if(energy_cycle2<0) energy_cycle2*=-1;
        energy_total2+=energy_cycle2;
        energy_cycle=0;
        energy_cycle2=0;
        }
cycle_started=1;
triggered=1;
}
// Reset filter trap after the voltage goes up
if(v>VOLTAGE_LOW_THRESHOLD) triggered=0;
v_last=v;

    if(cycle_started==1)
        {
        ticks++;
        if(c1<c_p2p_low) c_p2p_low=c1;
        if(c1>c_p2p_high) c_p2p_high=c1;
        if(v<v_p2p_low) v_p2p_low=v;
        if(v>v_p2p_high) v_p2p_high=v;
        c1-=c1_center;

        if(c2<c_p2p_low2) c_p2p_low2=c2;
        if(c2>c_p2p_high2) c_p2p_high2=c2;
        c2-=c1_center;


        // remove noise at floor
        //if(c1<10) c1=0;

        current_total+=((int32_t)c1*(int32_t)c1);
        voltage_total+=((int32_t)v*(int32_t)v);
        if(cycle_state==CYCLE_LOW) v*=-1;
        energy_cycle+=((int32_t)c1*(int32_t)v);

        current_total2+=((int32_t)c2*(int32_t)c2);
        energy_cycle2+=((int32_t)c2*(int32_t)v);
        //printf( "c1=%u current=%lu\r\n",c1, current_total );
        if(ticks>=1000)
                        {

                        tmp_d=current_total / ticks;
                        if(tmp_d<0) tmp_d=0;
                        tmp_d=sqrt(tmp_d);
                        rms_current=(uint16_t)tmp_d;

                        tmp_d=current_total2 / ticks;
                        if(tmp_d<0) tmp_d=0;
                        tmp_d=sqrt(tmp_d);
                        rms_current2=(uint16_t)tmp_d;

                        tmp_d=voltage_total / ticks;
                        if(tmp_d<0) tmp_d=0;
                        tmp_d=sqrt(tmp_d);
                        rms_voltage=(uint16_t)tmp_d;

                        if(energy_total<0) energy_total=0;
                        true_power=energy_total / ticks;
                        if(true_power>300) cummulative_energy+=true_power;

                        if(energy_total2<0) energy_total2=0;
                        true_power2=energy_total2 / ticks;
                        if(true_power2>300) cummulative_energy2+=true_power2;

                        total_secs++;
                        // FIXME: divide by time
                        tmp_energy=cummulative_energy;
                        tmp_energy2=cummulative_energy2;
                        // Auto-center to acount for thermal drift
                        //c1_center=(c_p2p_high-c_p2p_low)/2;   
                        //v1_center=(v_p2p_high-v_p2p_low)/2;   
                        l_v_p2p_high=v_p2p_high;
                        l_v_p2p_low=v_p2p_low;
                        l_c_p2p_high=c_p2p_high;
                        l_c_p2p_low=c_p2p_low;

                        l_c_p2p_high2=c_p2p_high2;
                        l_c_p2p_low2=c_p2p_low2;
                        if(cycle_cnt==0) nrk_led_toggle(RED_LED);
                        else
                           cycle_avg=ticks/cycle_cnt;



                        freq=cycle_cnt;
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
                        }

        }

}

void power_init ()
{
  ADC_INIT ();
  ADC_ENABLE ();
  ADC_VREF_VCC();
  nrk_timer_int_configure( NRK_APP_TIMER_0, 1, 7373, calc_power);
  ticks=0;
  cycle_state=CYCLE_HIGH;
  //cycle_state_last=CYCLE_UNKNOWN;
  cycle_cnt=0;
  cycle_avg=0;
  cycle_started=0;
  c1_center=496;
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
  cummulative_energy2=0;
  cummulative_energy=0;
  total_secs=0;
  v_last=VOLTAGE_LOW_THRESHOLD+10;
  triggered=0;

  nrk_gpio_direction(NRK_DEBUG_2, NRK_PIN_OUTPUT);

  //startup_sock_state=nrk_eeprom_read_byte(0x100);
  //if((startup_sock_state&0x01)==0x01)
  //      {
                nrk_timer_int_start(NRK_APP_TIMER_0);
                power_mon_enable();
                socket_0_enable();
                socket_0_active=1;
  //      }
  //else {
  //socket_0_active=0;
  //power_mon_disable();
  //              nrk_kprintf( PSTR("Socket inactive\r\n"));
  //}
  //if((startup_sock_state&0x02)==0x02)
  //      {
                socket_1_enable();
                socket_1_active=1;
  //      }
  // else {
  // socket_1_active=0;
  // }



}
