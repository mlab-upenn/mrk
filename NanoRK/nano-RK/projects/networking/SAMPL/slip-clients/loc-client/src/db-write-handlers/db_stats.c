#include <stdlib.h>
#include <time.h>
#include <sampl.h>
#include <globals.h>
#include <transducer_pkt.h>
#include <transducer_registry.h>
#include <stats_pkt.h>
#include <tx_queue.h>
#include <error_log.h>
#include <ffdb.h>

#if SOX_SUPPORT
#include <xmpp_transducer.h>
#endif



void db_write_stats_pkt (SAMPL_GATEWAY_PKT_T * gw_pkt)
{
uint8_t i;
int8_t v,type,value;
static struct firefly_stats ff_stats;
char mac_id_name[32];
char type_str[32];
uint32_t c_mac;
STATS_PKT_T stats;

if(gw_pkt->pkt_type!=STATS_PKT) return 0;
        // GW reply packets do not include the tran_pkt header
        // since this information can be captured in the gw
        // packet header info.  So fill out a dummy packet

for(i=0; i<gw_pkt->num_msgs; i++ )
		{
		  stats_pkt_get( &stats, gw_pkt->payload, i);
      c_mac =
          gw_pkt->subnet_mac[2] << 24 | gw_pkt->
          subnet_mac[1] << 16 | gw_pkt->subnet_mac[0] << 8 | stats.mac_addr;
       	sprintf(mac_id_name,"%02x%02x%02x%02x_stats",gw_pkt->subnet_mac[2],gw_pkt->subnet_mac[1],gw_pkt->subnet_mac[0],stats.mac_addr);
		  printf( "Stats pkt from 0x%x\n",stats.mac_addr );
		  printf( "  tx pkts %d\n",stats.tx_pkts);
		  printf( "  rx pkts %d\n",stats.rx_pkts);
		  printf( "  rx failures %d\n",stats.rx_failures);
		  printf( "  tx retry pkts %d\n",stats.tx_retry);
		  printf( "  uptime %d sec\n",stats.uptime);
		  printf( "  deep sleep %d sec\n",stats.deep_sleep);
		  printf( "  idle time %d sec\n",stats.idle_time);
		  printf( "  sensor samples %d\n",stats.sensor_samples);
			ff_stats.id=mac_id_name;
			ff_stats.time=time(NULL);
			ff_stats.tx_pkts=stats.tx_pkts;
			ff_stats.rx_pkts=stats.rx_pkts;
			ff_stats.uptime=stats.uptime;
			ff_stats.deep_sleep=stats.deep_sleep;
			ff_stats.idle_time=stats.idle_time;
			ff_stats.sensor_samples=stats.sensor_samples;
		  write_ff_stats(ff_stats);
		}
	

}
