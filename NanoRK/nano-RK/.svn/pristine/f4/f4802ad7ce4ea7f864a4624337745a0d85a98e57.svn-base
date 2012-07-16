#include <globals.h>
#include <nrk.h>
#include <sampl.h>
#include <transducer_handler.h>
#include <transducer_registry.h>
#include "power_vars.h"
#include <nrk_timer.h>
#include <ff_power.h>
#include <eeprom_state.h>

static nrk_time_t tmp_time;

static FF_POWER_SENSE_PKT ps;
static FF_POWER_RQST_PKT pr;
static FF_POWER_ACTUATE_PKT pa;
static FF_POWER_DEBUG_PKT pd;
static FF_POWER_CONFIG_PKT pc;

int8_t transducer_handler(TRANSDUCER_MSG_T *in_msg, TRANSDUCER_MSG_T *out_msg)
{
uint8_t i;
uint8_t v;
/*
nrk_kprintf( PSTR("in_msg\r\n  mac_addr=") ); printf( "%d\r\n", in_msg->mac_addr );
nrk_kprintf( PSTR("  type=") ); printf( "%d\r\n", in_msg->type);
nrk_kprintf( PSTR("  len=") ); printf( "%d\r\n", in_msg->len);
nrk_kprintf( PSTR("  payload= [ " ));
for(i=0; i<in_msg->len; i++ )
	printf( "%d ",in_msg->payload[i] );
nrk_kprintf( PSTR( "]\r\n" ));
*/

v=0;
if(in_msg->type==TRAN_POWER_PKT)
{
switch(in_msg->payload[0])
{
	case ACTUATE_PKT:
		out_msg->type=TRAN_NCK;
		out_msg->len=0;
		ff_power_actuate_unpack( in_msg->payload, &pa );	
		if( pa.socket0_state==SOCKET_ON)
			{
  			nrk_timer_int_start(NRK_APP_TIMER_0);
			power_mon_enable();
			//nrk_eeprom_write_byte(0x100,(socket_1_active<<1 | 0x1));
			socket_0_enable();	
			socket_0_active=1;
			write_eeprom_flag|=0x2;
			//nrk_kprintf( PSTR( "S0 enabled\r\n" ));
			out_msg->type=TRAN_ACK;
			v=1;
			}
		if( pa.socket0_state==SOCKET_OFF)
			{
  			nrk_timer_int_start(NRK_APP_TIMER_0);
			//nrk_eeprom_write_byte(0x100,(socket_1_active));
			socket_0_disable();
			socket_0_active=0;
			write_eeprom_flag|=0x2;
			//nrk_kprintf( PSTR( "S0 disabled\r\n" ));
			rms_current=0;
			true_power=0;		
			out_msg->type=TRAN_ACK;
			v=1;
			}
		if( pa.socket1_state==SOCKET_ON)
			{
  			nrk_timer_int_start(NRK_APP_TIMER_0);
			power_mon_enable();
			//nrk_eeprom_write_byte(0x100,( (1<<1) | socket_0_active));
			socket_1_enable();	
			socket_1_active=1;
			write_eeprom_flag|=0x2;
			//nrk_kprintf( PSTR( "S1 enabled\r\n" ));
			out_msg->type=TRAN_ACK;
			v=1;
			}
		if( pa.socket1_state==SOCKET_OFF)
			{
			nrk_timer_int_start(NRK_APP_TIMER_0);
			//nrk_eeprom_write_byte(0x100,(socket_0_active));
			socket_1_disable();
			socket_1_active=0;
			write_eeprom_flag|=0x2;
			//nrk_kprintf( PSTR( "S1 disabled\r\n" ));
			rms_current2=0;
			true_power2=0;		
			out_msg->type=TRAN_ACK;
			v=1;
			}
		break;

	case SENSE_RQST_PKT:
			
		//nrk_kprintf( PSTR("Sense request: "));
		write_eeprom_flag|=0x1;
		//energy_eeprom_write();
		out_msg->type=TRAN_POWER_PKT;
		ff_power_rqst_unpack( in_msg->payload, &pr );	
		if(pr.pkt_type==SENSE_PKT)
		{
		ps.type=SENSE_PKT;
		     if(pr.socket==0)
			{
			//nrk_kprintf( PSTR("sock 0\r\n"));
			ps.socket_num=0;
			ps.socket0_state= socket_0_active;
			ps.socket1_state= socket_1_active;
			ps.rms_current=rms_current;
			ps.rms_voltage=rms_voltage;
			ps.true_power=true_power;
			for(i=0; i<6; i++ ) ps.energy[i]=tmp_energy.byte[i];
			}
		     if(pr.socket==1)
			{
			//nrk_kprintf( PSTR("sock 1\r\n"));
			ps.socket_num=1;
			ps.socket0_state= socket_0_active;
			ps.socket1_state= socket_1_active;
			ps.rms_current=rms_current2;
			ps.rms_voltage=rms_voltage;
			ps.true_power=true_power2;
			for(i=0; i<6; i++ ) ps.energy[i]=tmp_energy2.byte[i];
			}
			out_msg->len=ff_power_sense_pack(out_msg->payload, &ps);
			v=1;
		}
		if(pr.pkt_type==DEBUG_PKT )
		{
			//nrk_kprintf( PSTR("debug\r\n"));
			write_eeprom_flag|=0x1;
			//energy_eeprom_write();
			pd.type=DEBUG_PKT;
			pd.rms_current=rms_current;
			pd.rms_current2=rms_current2;
			pd.rms_voltage=rms_voltage;
			pd.true_power=true_power;
			pd.true_power2=true_power2;
			pd.freq=freq;
			for(i=0; i<6; i++ ) pd.energy[i]=tmp_energy.byte[i];
			for(i=0; i<6; i++ ) pd.energy2[i]=tmp_energy2.byte[i];
			pd.current_p2p_high=l_c_p2p_high;
			pd.current_p2p_low=l_c_p2p_low;
			pd.current_p2p_high2=l_c_p2p_high2;
			pd.current_p2p_low2=l_c_p2p_low2;
        		pd.voltage_p2p_high=l_v_p2p_high;
        		pd.voltage_p2p_low=l_v_p2p_low;
			pd.total_secs=total_secs;
			pd.socket0_state= socket_0_active;
			pd.socket1_state= socket_1_active;
			out_msg->len=ff_power_debug_pack(out_msg->payload, &pd);
			v=1;
		}
	
	break;

	case CONFIG_PKT:
			out_msg->type=TRAN_ACK;
			out_msg->len=0;
			ff_power_config_unpack( in_msg->payload, &pc );	
			if(pc.socket==0)
				{
				socket_0_push_enabled=pc.push_enable;
				socket_0_push_threshold=pc.push_threshold;
				write_eeprom_flag|=0x2;
				}
			if(pc.socket==1)
				{
				socket_1_push_enabled=pc.push_enable;
				socket_1_push_threshold=pc.push_threshold;
				write_eeprom_flag|=0x2;
				}
			if(pc.socket>1) out_msg->type=TRAN_NCK;
			v=1;
	break;

	case RESET_PKT: 
			energy_eeprom_erase();
			out_msg->type=TRAN_ACK;
			out_msg->len=0;
			write_eeprom_flag|=0x1;
			v=1;
	break;

}

}

return v;
}
