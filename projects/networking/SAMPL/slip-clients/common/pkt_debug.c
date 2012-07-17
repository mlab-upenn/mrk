#include <sampl.h>
#include <pkt_debug.h>
#include <ff_basic_sensor_pkt.h>
#include <transducer_pkt.h>
#include <ff_power.h>
#include <transducer_registry.h>
#include <ff_power.h>
#include <ack_pkt.h>
#include <stats_pkt.h>
#include <neighbor_pkt.h>
#include <eeprom_data.h>

// Battery capacity mAH
#define BATTERY_CAPACITY	20000
// Idle cpu in ma 
#define IDLE_CPU	1.0
// Active cpu in ma 
#define ACTIVE_CPU	6.0
// RX in ma 
#define RADIO_RX	19.0
// TX in ma 
#define RADIO_TX	18.0	
// check rate in ms 
#define LPL_RATE	100	
// check time in ms 
#define LPL_CHECK_TIME	2	
static float avg_current;
static float cpu_avg_current;
static float radio_avg_current;
static float radio_checks;

static unsigned long long tmp_energy;

#define gw_mac		0


void print_gw_packet_elements(SAMPL_GATEWAY_PKT_T *gw_pkt)
{
int i;
ACK_PKT_T ack;
EEPROM_STORAGE_PKT_T data_pkt;
STATS_PKT_T stats;
NLIST_PKT_T nlist;
FF_POWER_SENSE_PKT ps;
FF_POWER_DEBUG_PKT pd;
FF_POWER_ACTUATE_PKT pa;
TRANSDUCER_PKT_T tran_pkt;
TRANSDUCER_MSG_T tran_msg;
FF_SENSOR_SHORT_PKT_T sensor_short;


unpack_gateway_packet(gw_pkt );
	// You will have a gateway packet here to operate on.
	// The gateway packet has a payload which contains user defined packets.
	
	// Lets print the raw packet:
	print_gw_packet(gw_pkt);

	// Now lets parse out some application data:
	switch(gw_pkt->pkt_type)
	{
	// PING and ACK are the same
	case PING_PKT:
	case ACK_PKT:
		for(i=0; i<gw_pkt->num_msgs; i++ )
		{
		  ack_pkt_get( &ack, gw_pkt->payload, i);
		  printf( "Ack pkt from 0x%x\n",ack.mac_addr );
		}
		break;
	case DATA_STORAGE_PKT:
		for(i=0; i<gw_pkt->num_msgs; i++ )
		{
		  eeprom_storage_pkt_unpack( &data_pkt, gw_pkt->payload);
		  printf( "EEPROM data from 0x%x\n",data_pkt.mac);
		  printf( "   mode 0x%x\n",data_pkt.mode);
		  printf( "   addr 0x%x\n",data_pkt.addr);
		  printf( "   length 0x%x\n",(uint8_t)data_pkt.data_len);
		  printf( "   data: [");
		  if(data_pkt.mode!=EE_READ)
			{
		  	for(i=0; i<(uint8_t)data_pkt.data_len; i++)
				printf( "0x%x ",(uint8_t)data_pkt.eeprom_payload[i] );
		  	printf( "]\n" );	
			}
		}
		break;
	
	case STATS_PKT:
		for(i=0; i<gw_pkt->num_msgs; i++ )
		{
		  stats_pkt_get( &stats, gw_pkt->payload, i);
		  printf( "Stats pkt from 0x%x\n",stats.mac_addr );
		  printf( "  0x%x tx pkts %d\n",stats.mac_addr,stats.tx_pkts);
		  printf( "  0x%x rx pkts %d\n",stats.mac_addr,stats.rx_pkts);
		  printf( "  0x%x rx failures %d\n",stats.mac_addr,stats.rx_failures);
		  printf( "  0x%x tx retry pkts %d\n",stats.mac_addr,stats.tx_retry);
		  printf( "  0x%x uptime %d sec\n",stats.mac_addr,stats.uptime);
		  printf( "  0x%x deep sleep %d sec\n",stats.mac_addr,stats.deep_sleep);
		  printf( "  0x%x idle time %d sec\n",stats.mac_addr,stats.idle_time);
		  printf( "  0x%x sensor samples %d\n",stats.mac_addr,stats.sensor_samples);
		  avg_current=0;
		  radio_avg_current=0;
		  cpu_avg_current=0;
		  cpu_avg_current=(((stats.idle_time-stats.deep_sleep)*IDLE_CPU) + ((stats.uptime-stats.idle_time)*ACTIVE_CPU))/(float)stats.uptime; 
		  radio_checks = ((float)stats.uptime*1000)/((float)LPL_RATE);
		  radio_avg_current=(((stats.tx_pkts+stats.tx_retry)*LPL_RATE*RADIO_TX)+((stats.rx_pkts+stats.rx_failures)*LPL_RATE*RADIO_RX)+(radio_checks*LPL_CHECK_TIME*RADIO_RX))/((float)stats.uptime*1000); 
		  avg_current=radio_avg_current+cpu_avg_current;
		  printf( "  0x%x Radio current: %f\n",stats.mac_addr, radio_avg_current );
		  printf( "  0x%x CPU current: %f\n",stats.mac_addr, cpu_avg_current );
		  printf( "  0x%x Avg current: %f\n",stats.mac_addr, avg_current );
		  printf( "  0x%x Estimated Life: %f days\n", stats.mac_addr,(BATTERY_CAPACITY/avg_current)/24 );
		}
		break;
  
	case SUBNET_NEIGHBOR_LIST_PKT:
                for(i=0; i<gw_pkt->num_msgs; i++ )
                {
                  nlist_pkt_get( &nlist, gw_pkt->payload, i);
                  printf( "Nlist pkt from 0x%x\n",nlist.mac_addr );
                  printf( "  Neighbor: 0x%x\n",nlist.neighbor_mac);
                  printf( "  RSSI: %d\n",nlist.rssi);
                }
                break;

	case TRANSDUCER_PKT:
                transducer_pkt_unpack(&tran_pkt, gw_pkt->payload );
                printf( "Transducer pkt with %d msgs\n",tran_pkt.num_msgs );
                for(i=0; i<tran_pkt.num_msgs; i++ )
                {
                transducer_msg_get(&tran_pkt, &tran_msg, i );
                switch(tran_msg.type)
                {
                case TRAN_POWER_PKT:
                        if(tran_msg.payload[0]==ACTUATE_PKT)
                        {
                        ff_power_actuate_unpack( tran_msg.payload, &pa);
			printf( "Power Actuate Packet for %u\n",tran_msg.mac_addr );
                        printf( "\tSocket0: ");
												if(pa.socket0_state==SOCKET_HOLD) printf( "SOCKET_HOLD\n" );
												if(pa.socket0_state==SOCKET_ON) printf( "SOCKET_ON\n" );
												if(pa.socket0_state==SOCKET_OFF) printf( "SOCKET_OFF\n" );
                        printf( "\tSocket1: ",pa.socket1_state);
												if(pa.socket1_state==SOCKET_HOLD) printf( "SOCKET_HOLD\n" );
												if(pa.socket1_state==SOCKET_ON) printf( "SOCKET_ON\n" );
												if(pa.socket1_state==SOCKET_OFF) printf( "SOCKET_OFF\n" );
                        } else if(tran_msg.payload[0]==SENSE_PKT)
                        {
                        ff_power_sense_unpack( tran_msg.payload, &ps);
                        printf( "Sense pkt for Socket %d from %u\n",ps.socket_num,tran_msg.mac_addr );
                        printf( "\tsocket0 state: %d\n",ps.socket0_state);
                        printf( "\tsocket1 state: %d\n",ps.socket1_state);
                        printf( "\trms_current: %d\n",ps.rms_current);
                        printf( "\trms_voltage: %d\n",ps.rms_voltage);
                        printf( "\ttrue_power: %d\n",ps.true_power);
			
			tmp_energy=ps.energy[5]<<40 | ps.energy[4]<<32 | ps.energy[3]<<24 | ps.energy[2]<<16 | ps.energy[1]<<8 | ps.energy[0];
                        printf( "\tenergy: %llu\n",tmp_energy);
                        } else
                        if(tran_msg.payload[0]==DEBUG_PKT)
                        {
                        ff_power_debug_unpack( tran_msg.payload, &pd);
			printf( "Power Debug Pkt from %u\n",tran_msg.mac_addr );
                        printf( "\trms_voltage (all): %d\n",pd.rms_voltage);
                        printf( "\tfreq (all): %d\n",pd.freq);
                        printf( "\trms_current: %d\n",pd.rms_current);
                        printf( "\ttrue_power: %d\n",pd.true_power);
			tmp_energy=pd.energy[5]<<40 | pd.energy[4]<<32 | pd.energy[3]<<24 | pd.energy[2]<<16 | pd.energy[1]<<8 | pd.energy[0];
                        printf( "\tenergy: %llu\n",tmp_energy);
                        printf( "\trms_current2: %d\n",pd.rms_current2);
                        printf( "\ttrue_power2: %d\n",pd.true_power2);
			tmp_energy=pd.energy2[5]<<40 | pd.energy2[4]<<32 | pd.energy2[3]<<24 | pd.energy2[2]<<16 | pd.energy2[1]<<8 | pd.energy2[0];
                        printf( "\tenergy2: %llu\n",tmp_energy);
                        printf( "\ttime: %d\n",pd.total_secs);
                        }
                                else
                        printf( "Unkown Power msg type %d\n", tran_msg.payload[0]);

                break;
		case TRAN_FF_BASIC_SHORT:
        		ff_basic_sensor_short_unpack (&tran_msg, &sensor_short);
        		printf ("Sensor pkt from 0x%x\n", tran_msg.mac_addr);
        		printf ("   Light: %d\n", sensor_short.light);
        		printf ("   Temperature: %d\n", sensor_short.temperature);
        		printf ("   Acceleration: %d\n", sensor_short.acceleration);
        		printf ("   Sound Level: %d\n", sensor_short.sound_level);
        		printf ("   Battery: %d\n", sensor_short.battery + 100);
        		break;
      		case TRAN_ACK:
        		printf( "Transducer ACK from 0x%x\n",tran_msg.mac_addr );
        		break;
      		case TRAN_NCK:
        		printf( "Transducer NCK from 0x%x\n",tran_msg.mac_addr );
        		break;

		default:
                   printf( "Unkown Transducer msg type %d\n", tran_msg.type );


                }

                }

                break;



	}
}



void print_ds_packet(SAMPL_DOWNSTREAM_PKT_T *ds_pkt )
{
int i;
	printf( "Downstream Packet Header info:\n" ); 
	printf( "  pkt type\t\t0x%x\n",ds_pkt->pkt_type);
	printf( "  ctrl flags\t\t0x%x\n",ds_pkt->ctrl_flags );
	printf( "  seq num\t\t0x%x\n",ds_pkt->seq_num );
	printf( "  priority\t\t0x%x\n",ds_pkt->priority);
	printf( "  ack retry\t\t0x%x\n",ds_pkt->ack_retry);
	printf( "  subnet mac\t\t0x%x\n",ds_pkt->subnet_mac[0]);
	printf( "  hop_cnt\t\t0x%x\n",ds_pkt->hop_cnt);
	printf( "  hop_max\t\t0x%x\n",ds_pkt->hop_max);
	printf( "  delay_per_level\t%d seconds\n",ds_pkt->delay_per_level);
	printf( "  nav\t\t\t%d seconds\n",ds_pkt->nav);
	printf( "  mac_check_rate\t%d ms\n",ds_pkt->mac_check_rate);
	printf( "  rssi_threshold\t\t\t%d\n",(int8_t)ds_pkt->rssi_threshold);
	printf( "  last_hop_mac\t\t0x%x\n",ds_pkt->last_hop_mac);
	printf( "  mac_filter_num\t0x%x\n",ds_pkt->mac_filter_num);
	printf( "  aes_ctr\t\t0x%x 0x%x 0x%x 0x%x\n",ds_pkt->aes_ctr[3], ds_pkt->aes_ctr[3], ds_pkt->aes_ctr[2], ds_pkt->aes_ctr[1], ds_pkt->aes_ctr[0]);
	printf( "Mac Filter List: " );
        for(i=0; i<ds_pkt->mac_filter_num; i++ )
		printf( "0x%x ",ds_pkt->buf[DS_PAYLOAD_START+i] );
	printf( "\n\n" );
	printf( "Payload Data: " );
        for(i=0; i<ds_pkt->payload_len; i++ )
		printf( "0x%x ",ds_pkt->payload[i] );
	printf( "\n\n" );


}

void print_gw_packet (SAMPL_GATEWAY_PKT_T * gw_pkt)
{
  int i;
  printf ("Gateway Packet Header info:\n");
  printf ("  pkt type\t\t0x%x\n", gw_pkt->pkt_type);
  printf ("  ctrl flags\t\t0x%x\n", gw_pkt->ctrl_flags);
  printf ("  seq num\t\t0x%x\n", gw_pkt->seq_num);
  printf ("  priority\t\t0x%x\n", gw_pkt->priority);
  printf ("  ack retry\t\t0x%x\n", gw_pkt->ack_retry);
  printf ("  subnet mac\t\t0x%x\n", gw_pkt->subnet_mac[0]);
  printf ("  src mac\t\t0x%x\n", gw_pkt->src_mac);
  printf ("  dst mac\t\t0x%x\n", gw_pkt->dst_mac);
  printf ("  last hop mac\t\t0x%x\n", gw_pkt->last_hop_mac);
  printf ("  error code\t\t0x%x\n", gw_pkt->error_code);
  printf ("  rssi\t\t%d\n", gw_pkt->rssi);
  printf ("Payload Data [%d]: ",gw_pkt->payload_len);
  for (i = 0; i < gw_pkt->payload_len; i++)
    printf ("0x%x ", gw_pkt->payload[i]);
  printf ("\n\n");

  if(gw_pkt->error_code!=0)
        {
                printf( "  *****************************************\n" );
                printf( "  Error Codes:\n" );
        if((gw_pkt->error_code&HOP_ERROR_MASK) !=0 )
                printf( "\t\tHOP_ERROR_MASK\n" );
        if((gw_pkt->error_code&NAV_ERROR_MASK) !=0 )
                printf( "\t\tNAV_ERROR_MASK\n" );
        if((gw_pkt->error_code&DELAY_PER_LEVEL_ERROR_MASK) !=0 )
                printf( "\t\tDELAY_PER_LEVEL_ERROR_MASK\n" );
        if((gw_pkt->error_code&MAX_HOPS_ERROR_MASK) !=0 )
                printf( "\t\tMAX_HOPS_ERROR_MASK\n" );
        if((gw_pkt->error_code&OVERFLOW_ERROR_MASK) !=0 )
                printf( "\t\tOVERFLOW_ERROR_MASK\n" );
        if((gw_pkt->error_code&UNKNOWN_ERROR_MASK) !=0 )
                printf( "\t\tUNKNOWN_ERROR_MASK\n" );
        if((gw_pkt->error_code&INVALID_DATA_ERROR_MASK) !=0 )
                printf( "\t\tINVALID_DATA_ERROR_MASK\n" );
                printf( "  *****************************************\n\n" );
        }

}



void error(char *msg)
{
  perror(msg);
  exit(0);
}



