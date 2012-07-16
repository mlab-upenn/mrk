#include <event_detector.h>
#include <nrk.h>
#include <transducer_handler.h>
#include <transducer_registry.h>
#include <ff_power.h>
#include <power_vars.h>
#include <eeprom_state.h>

static uint8_t async_buf[64];
static FF_POWER_DEBUG_PKT pd_pkt;

static TRANSDUCER_MSG_T tran_msg;
static TRANSDUCER_PKT_T tran_pkt;

static uint32_t true_power_last, true_power_last2;

static nrk_sig_mask_t my_sigs;

static uint8_t len,val; 
static uint16_t delta,delta2;
//static nrk_time_t ev_timeout;

void event_detector_task()
{

update_energy_sig=nrk_signal_create();
if(update_energy_sig==NRK_ERROR)
	nrk_kprintf(PSTR("Error creating update_energy_sig signal!\r\n"));

nrk_signal_register(update_energy_sig);
if(v==NRK_ERROR) nrk_kprintf( PSTR( "nrk_signal_register failed\r\n" ));

while (!sampl_started())
    nrk_wait_until_next_period ();
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

  if(total_secs%400==0 || write_eeprom_flag!=0)
	{
	// write to eeprom
	if((write_eeprom_flag&0x2)!=0) 
	{
		nrk_int_disable();
		nrk_eeprom_write_byte(0x100,(socket_1_push_enabled<<3 | socket_0_push_enabled<<2 | socket_1_active<<1 | socket_0_active));
		nrk_eeprom_write_byte(EEPROM_PUSH_THRESHOLD_0,socket_0_push_threshold);
		nrk_eeprom_write_byte(EEPROM_PUSH_THRESHOLD_1,socket_1_push_threshold);
		nrk_int_enable();
	}
	if((write_eeprom_flag&0x1)!=0) energy_eeprom_write();
	write_eeprom_flag=0;
	}


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

true_power_last=true_power;
true_power_last2=true_power2;


} 
// toggle RED led if freq not above 50Hz
if(freq==0) nrk_led_toggle(RED_LED);


}



}

