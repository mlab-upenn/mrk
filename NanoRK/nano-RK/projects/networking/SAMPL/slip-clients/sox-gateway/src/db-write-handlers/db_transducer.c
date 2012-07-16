#include <stdlib.h>
#include <time.h>
#include <sampl.h>
#include <globals.h>
#include <ff_basic_sensor_pkt.h>
#include <ff_basic_sensor_cal.h>
#include <jiga_watt_cal.h>
#include <transducer_pkt.h>
#include <transducer_registry.h>
#include <ff_power.h>
#include <tx_queue.h>
#include <error_log.h>
#include <ffdb.h>

#if SOX_SUPPORT
#include <xmpp_transducer.h>
#endif

#define TEMPERATURE_OFFSET 400


static uint8_t freq;
static uint16_t adc_rms_current, adc_rms_voltage;
static uint16_t adc_rms_current2;
static uint32_t adc_true_power, adc_energy, time_secs;
static uint32_t adc_true_power2, adc_energy2;
static uint16_t adc_v_p2p_low, adc_v_p2p_high;
static uint16_t adc_i_p2p_low, adc_i_p2p_high;
static uint16_t adc_i_p2p_low2, adc_i_p2p_high2;
static unsigned long long tmp_energy;

static float cal_energy, rms_voltage, rms_current, true_power, apparent_power, power_factor;
static float rms_current2, true_power2, apparent_power2, power_factor2;

static jiga_watt_cal_struct_t jiga_watt_cal;



void db_write_transducer_pkt (SAMPL_GATEWAY_PKT_T * gw_pkt)
{
uint8_t i;
int8_t v,type,value;
static struct firefly_env ff_env;
static struct power_meter ff_pow;
static struct generic_integer_sensor ff_binary;
char mac_id_name[32];
char type_str[32];
FF_SENSOR_SHORT_PKT_T sensor_short;
TRANSDUCER_PKT_T tp;
TRANSDUCER_MSG_T tm;
FF_POWER_SENSE_PKT ps;
FF_POWER_DEBUG_PKT pd;
FF_POWER_VIRTUAL_PKT pv;
float temp_cal, battery_cal;
uint16_t light_cal;
uint32_t c_mac;

if(gw_pkt->pkt_type!=TRANSDUCER_PKT) return 0;
        // GW reply packets do not include the tran_pkt header
        // since this information can be captured in the gw
        // packet header info.  So fill out a dummy packet
    v=transducer_pkt_unpack(&tp, gw_pkt->payload );
    if(v!=1) {
        printf( "Transducer checksum failed!\n" );
				return 0;
        }
    for (i = 0; i < tp.num_msgs; i++) {
      v=transducer_msg_get (&tp,&tm, i);
      printf( "tm_mac=%u\n",tm.mac_addr);
      switch (tm.type) {
      case TRAN_FF_BASIC_SHORT:
        ff_basic_sensor_short_unpack (&tm, &sensor_short);
 
        c_mac =
          gw_pkt->subnet_mac[2] << 24 | gw_pkt->
          subnet_mac[1] << 16 | gw_pkt->subnet_mac[0] << 8 | tm.mac_addr;

	// calibrate temperature
        temp_cal =
          ff_basic_sensor_cal_get_temp ((sensor_short.temperature << 1) +
                                        TEMPERATURE_OFFSET) +
          ff_basic_sensor_cal_get_temp_offset (c_mac);
        
	// calibrate light
        printf ("battery=%u light=%u\n", sensor_short.battery,
                sensor_short.light);
        light_cal =
          ff_basic_sensor_cal_get_light (sensor_short.light,
                                         ((float)
                                          (sensor_short.battery +
                                           100)) / 100) +
          ff_basic_sensor_cal_get_light_offset (c_mac);
        light_cal = 255 - light_cal;

	// adjust battery	
        battery_cal = ((float) (sensor_short.battery + 100)) / 100;
        


        printf ("Sensor pkt from 0x%x\n", tm.mac_addr);
        printf ("   Light: %u (%u)\n", light_cal, (uint8_t)sensor_short.light);
        printf ("   Temperature: %f (%u)\n", temp_cal, (uint8_t)sensor_short.temperature);
        printf ("   Acceleration: %u\n", (uint8_t)sensor_short.acceleration);
        printf ("   Sound Level: %u\n", (uint8_t)sensor_short.sound_level);
        printf ("   Battery: %f (%u)\n",battery_cal, (uint8_t)sensor_short.battery + 100);
       	sprintf(mac_id_name,"%02x%02x%02x%02x_env",gw_pkt->subnet_mac[2],gw_pkt->subnet_mac[1],gw_pkt->subnet_mac[0],tm.mac_addr);
	ff_env.id=mac_id_name;
	ff_env.time=time(NULL);
	ff_env.light=(int)light_cal;
	ff_env.temp=(int)temp_cal;
	ff_env.accl=sensor_short.acceleration;
	ff_env.voltage=(int)(battery_cal*100);
	ff_env.audio=sensor_short.sound_level;
	write_ff_env(ff_env);
        break;
      case TRAN_BINARY_SENSOR:
        printf( "BINARY value from 0x%x\n",tm.mac_addr );
				type=tm.payload[0]&0x7f;
				value=tm.payload[0]>>7;
				printf( "type: %u value: %u\n", type, value);
       	sprintf(mac_id_name,"%02x%02x%02x%02x",gw_pkt->subnet_mac[2],gw_pkt->subnet_mac[1],gw_pkt->subnet_mac[0],tm.mac_addr);

				switch(type)
				{
				case BINARY_SENSOR_MOTION:
					sprintf(type_str,"motion" );
					sprintf(mac_id_name,"%s_motion",mac_id_name);
					break;

				case BINARY_SENSOR_LEAK:
					sprintf(type_str,"liquid" );
					sprintf(mac_id_name,"%s_liquid",mac_id_name);
					break;

				case BINARY_SENSOR_GPIO:
					sprintf(type_str,"gpio" );
					sprintf(mac_id_name,"%s_gpio",mac_id_name);
					break;

					default:
						sprintf(mac_id_name,"%s_binary",mac_id_name);
						sprintf(type_str,"binary" );

				}

			ff_binary.id=mac_id_name;
			ff_binary.type=type_str;
		  	ff_binary.time=time(NULL);
			ff_binary.value=(tm.payload[0]>>7);
			write_generic_integer(ff_binary);
		break;

      case TRAN_ACK:
        printf( "Transducer ACK from 0x%x\n",tm.mac_addr );
        break;
       case TRAN_NCK:
        printf( "Transducer NCK from 0x%x\n",tm.mac_addr );
        break;

      case TRAN_POWER_PKT:
           if(tm.payload[0]==SENSE_PKT)
              {
                        ff_power_sense_unpack( tm.payload, &ps);
	
        		c_mac =
          			gw_pkt->subnet_mac[2] << 24 | gw_pkt->
          			subnet_mac[1] << 16 | gw_pkt->subnet_mac[0] << 8 | tm.mac_addr;
        		if (jiga_watt_cal_get (c_mac,ps.socket_num, &jiga_watt_cal) == 1) {
          			rms_voltage = (float)ps.rms_voltage / jiga_watt_cal.voltage_scaler;
          			rms_current = (float)ps.rms_current / jiga_watt_cal.current_scaler;
          			true_power = (float)ps.true_power / jiga_watt_cal.power_scaler;
				tmp_energy=ps.energy[5]<<40 | ps.energy[4]<<32 | ps.energy[3]<<24 | ps.energy[2]<<16 | ps.energy[1]<<8 | ps.energy[0];
				cal_energy=(float)tmp_energy / jiga_watt_cal.power_scaler;  // get Watt Seconds
				//cal_energy=cal_energy/3600; // convert to WHrs
        		}
        		else {
				cal_energy=0;
          			rms_voltage = 0;
          			rms_current = 0;
          			jiga_watt_cal.current_adc_offset = 0;
        		}

        		if (ps.rms_current <= jiga_watt_cal.current_adc_offset) {
          		// disconnected state
          		//adc_rms_current = 0;
          			rms_current = 0;
          			true_power = 0;
          			apparent_power = 0;
        		}

        		apparent_power = rms_voltage * rms_current;
        		//if (true_power > apparent_power)
        		//  apparent_power = true_power;
        		power_factor = true_power / apparent_power;

                        printf( "Socket %d:\n",ps.socket_num );
                        printf( "\tsocket0 state: %u\n",ps.socket0_state);
                        printf( "\tsocket1 state: %u\n",ps.socket1_state);
                        printf( "\trms_current: %f (%u)\n",rms_current,ps.rms_current);
                        printf( "\trms_voltage: %f (%u)\n",rms_voltage,ps.rms_voltage);
                        printf( "\ttrue_power: %f (%u)\n",true_power,ps.true_power);
        		printf ("\tappearent power: %f VA\n", apparent_power);
        		printf ("\tpower factor:  %f VA\n", power_factor);
                        printf( "\tenergy: %f (%llu)\n",cal_energy,tmp_energy);
       			sprintf(mac_id_name,"%02x%02x%02x%02x_%d_pwr",gw_pkt->subnet_mac[2],gw_pkt->subnet_mac[1],gw_pkt->subnet_mac[0],tm.mac_addr,ps.socket_num );
			ff_pow.id=mac_id_name;
			ff_pow.time=time(NULL);
			if(ps.socket_num==0)
				ff_pow.state=ps.socket0_state;
			else
				ff_pow.state=ps.socket1_state;
			ff_pow.rms_current=(int)(rms_current*100);
			ff_pow.rms_voltage=(int)rms_voltage;
			ff_pow.true_power=(int)true_power;
			ff_pow.energy=(int)cal_energy;
			write_power(ff_pow);
       } else if(tm.payload[0]==VIRTUAL_PKT)
		{
      ff_power_virtual_unpack( tm.payload, &pv);
			printf( "Got Virtual Packet:\n" );
			printf( "  state=%d\n",pv.state );
			printf( "  last state=%d\n",pv.last_state );
			printf( "  event=%d\n",pv.event);
			c_mac =
          			gw_pkt->subnet_mac[2] << 24 | gw_pkt->
          			subnet_mac[1] << 16 | gw_pkt->subnet_mac[0] << 8 | tm.mac_addr;


      if (jiga_watt_cal_get (c_mac,0, &jiga_watt_cal) == 1) {
          			rms_voltage = jiga_watt_cal.voltage_scaler;
          			rms_current = jiga_watt_cal.current_scaler;
          			true_power = jiga_watt_cal.power_scaler;
        			apparent_power = rms_voltage * rms_current;
        			power_factor = true_power / apparent_power;
				cal_energy=0;
				cal_energy= (pv.time[7]<<56) | (pv.time[6]<<48) | (pv.time[5]<<40) | (pv.time[4]<<32) | (pv.time[3]<<24) | (pv.time[2]<<16) | (pv.time[1]<<8) | pv.time[0];
				// convert to seconds (sampled at 4 hz)
				cal_energy=cal_energy/4;
				printf( "  time: %f\n",cal_energy );
				// convert to hours 
				cal_energy=cal_energy/3600;
				cal_energy=cal_energy*true_power;	
      sprintf(mac_id_name,"%02x%02x%02x%02x_0_pwr",gw_pkt->subnet_mac[2],gw_pkt->subnet_mac[1],gw_pkt->subnet_mac[0],tm.mac_addr);
			ff_pow.id=mac_id_name;
		  ff_pow.time=time(NULL);
		// Smooth the edges by using last state information
		  if(pv.last_state==0)
			{
			ff_pow.state=0;
			ff_pow.rms_current=0;
			ff_pow.rms_voltage=(int)rms_voltage;
			ff_pow.true_power=0;
			ff_pow.energy=(int)cal_energy;
			write_power(ff_pow);
			}

		  if(pv.last_state==1)
			{
			ff_pow.state=1;
			ff_pow.rms_current=(int)(rms_current*100);
			ff_pow.rms_voltage=(int)rms_voltage;
			ff_pow.true_power=(int)true_power;
			ff_pow.energy=(int)cal_energy;
			write_power(ff_pow);
			}

		  if(pv.state==0)
			{
  				true_power=0;
				rms_current=0;
				apparent_power=0;
			} 
			printf( "  voltage=%f\n",rms_voltage );
			printf( "  current=%f\n",rms_current);
			printf( "  true_power=%f\n",true_power);
			printf( "  energy=%f\n",cal_energy);
			ff_pow.state=pv.state;
			ff_pow.rms_current=(int)(rms_current*100);
			ff_pow.rms_voltage=(int)rms_voltage;
			ff_pow.true_power=(int)true_power;
			ff_pow.energy=(int)cal_energy;
			write_power(ff_pow);

        		}
 		} 
	else if(tm.payload[0]==DEBUG_PKT)
                 {
                        ff_power_debug_unpack( tm.payload, &pd);

        		c_mac =
          			gw_pkt->subnet_mac[2] << 24 | gw_pkt->
          			subnet_mac[1] << 16 | gw_pkt->subnet_mac[0] << 8 | tm.mac_addr;

        		if (jiga_watt_cal_get (c_mac,0, &jiga_watt_cal) == 1) {
          			rms_voltage = (float)pd.rms_voltage / jiga_watt_cal.voltage_scaler;
          			rms_current = (float)pd.rms_current / jiga_watt_cal.current_scaler;
          			true_power = (float)pd.true_power / jiga_watt_cal.power_scaler;
				tmp_energy=pd.energy[5]<<40 | pd.energy[4]<<32 | pd.energy[3]<<24 | pd.energy[2]<<16 | pd.energy[1]<<8 | pd.energy[0];
				cal_energy=(float)tmp_energy / jiga_watt_cal.power_scaler;  // get Watt Seconds
				//cal_energy=cal_energy/3600; // convert to WHrs
        		}
        		else {
				cal_energy=0;
          			rms_voltage = 0;
          			rms_current = 0;
          			jiga_watt_cal.current_adc_offset = 0;
        		}

        		if (pd.rms_current <= jiga_watt_cal.current_adc_offset) {
          		// disconnected state
          		//adc_rms_current = 0;
          			rms_current = 0;
          			true_power = 0;
          			apparent_power = 0;
        		}

        		apparent_power = rms_voltage * rms_current;
        		//if (true_power > apparent_power)
        		//  apparent_power = true_power;
        		power_factor = true_power / apparent_power;



       			sprintf(mac_id_name,"%02x%02x%02x%02x_0_pwr",gw_pkt->subnet_mac[2],gw_pkt->subnet_mac[1],gw_pkt->subnet_mac[0],tm.mac_addr);
			ff_pow.id=mac_id_name;
			ff_pow.time=time(NULL);
			ff_pow.state=pd.socket0_state;
			ff_pow.rms_current=(int)(rms_current*100);
			ff_pow.rms_voltage=(int)rms_voltage;
			ff_pow.true_power=(int)true_power;
			ff_pow.energy=(int)cal_energy; //pd.energy;
			write_power(ff_pow);

                        printf( "\trms_voltage (all): %f (%u)\n",rms_voltage, pd.rms_voltage);
                        printf( "\tfreq (all): %u\n",pd.freq);
                        printf( "\trms_current: %f (%u)\n",rms_current, pd.rms_current);
                        printf( "\ttrue_power: %f (%u)\n",true_power,pd.true_power);
                        printf( "\tenergy: %f (%llu)\n",cal_energy,tmp_energy);
                        printf( "\tstate: %u \n",pd.socket0_state);




        		if (jiga_watt_cal_get (c_mac,1, &jiga_watt_cal) == 1) {
          			rms_voltage = (float)pd.rms_voltage / jiga_watt_cal.voltage_scaler;
          			rms_current = (float)pd.rms_current2 / jiga_watt_cal.current_scaler;
          			true_power = (float)pd.true_power2 / jiga_watt_cal.power_scaler;
				tmp_energy=pd.energy2[5]<<40 | pd.energy2[4]<<32 | pd.energy2[3]<<24 | pd.energy2[2]<<16 | pd.energy2[1]<<8 | pd.energy2[0];
				cal_energy=(float)tmp_energy / jiga_watt_cal.power_scaler;  // get Watt Seconds
				//cal_energy=cal_energy/3600; // convert to WHrs
        		}
        		else {
				cal_energy=0;
          			rms_voltage = 0;
          			rms_current = 0;
          			jiga_watt_cal.current_adc_offset = 0;
        		}

        		if (pd.rms_current2 <= jiga_watt_cal.current_adc_offset) {
          		// disconnected state
          		//adc_rms_current = 0;
          			rms_current = 0;
          			true_power = 0;
          			apparent_power = 0;
        		}

        		apparent_power = rms_voltage * rms_current;
        		//if (true_power > apparent_power)
        		//  apparent_power = true_power;
        		power_factor = true_power / apparent_power;





       			sprintf(mac_id_name,"%02x%02x%02x%02x_1_pwr",gw_pkt->subnet_mac[2],gw_pkt->subnet_mac[1],gw_pkt->subnet_mac[0],tm.mac_addr);
			ff_pow.id=mac_id_name;
			ff_pow.time=time(NULL);
			ff_pow.state=pd.socket1_state;
			ff_pow.rms_current=(int)(rms_current*100);
			ff_pow.rms_voltage=(int)rms_voltage;
			ff_pow.true_power=(int)true_power;
			ff_pow.energy=(int)cal_energy;
			//ff_pow.energy=pd.energy2;
			write_power(ff_pow);

                        printf( "\trms_current2: %f (%u)\n",rms_current,pd.rms_current2);
                        printf( "\ttrue_power2: %f (%u)\n",true_power,pd.true_power2);
                        printf( "\tenergy2: %f (%llu)\n",cal_energy,tmp_energy);
                        printf( "\tstate2: %u \n",pd.socket1_state);
                        printf( "\ttime: %u\n",pd.total_secs);
               }
                 else
                 printf( "Unkown Power msg type %u\n", tm.payload[0]);

                break;


      default:
        printf ("unkown transducer packet: %u\n",tm.type);
      }
    }


}
