#include <stdlib.h>
#include <time.h>
#include <xmpp_transducer.h>
#include <sampl.h>
#include <soxlib.h>
#include <node_cache.h>
#include <globals.h>
#include <xmpp_pkt.h>
#include <ack_pkt.h>
#include <stats_pkt.h>
#include <transducer_pkt.h>
#include <xmpp_transducer.h>
#include <transducer_registry.h>
#include <xmpp_proxy.h>
#include <tx_queue.h>
#include <error_log.h>


void publish_xmpp_stats_pkt (SAMPL_GATEWAY_PKT_T * gw_pkt)
{
  uint8_t i;
  int8_t v;
  ACK_PKT_T ack;
  STATS_PKT_T stats;
  SOXMessage *msg = NULL;
  char sensor_raw_str[32];
  char sensor_adj_str[32];
  char sensor_type_str[32];
  char event_node[32];
  char reg_name[32];
  char time_str[100];
  char reg_id[100];
  int ret = 0, val;
  time_t timestamp;
	time (&timestamp);
  strftime (time_str, 100, "%Y-%m-%dT%X", localtime (&timestamp));


  if (gw_pkt->pkt_type != STATS_PKT)
    return 0;

for(i=0; i<gw_pkt->num_msgs; i++ )
		{
		  stats_pkt_get( &stats, gw_pkt->payload, i);

/*
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
*/

      sprintf (event_node, "%02x%02x%02x%02x", gw_pkt->subnet_mac[2],
               gw_pkt->subnet_mac[1], gw_pkt->subnet_mac[0], stats.mac_addr);


      printf ("event node=%s\n", event_node);


      check_and_create_node (event_node);
      val = reg_id_get (event_node, reg_id);
      if (val == 0)
        strcpy (reg_id, "unknown");
      printf ("    event node: %s reg_id: %s\n", event_node, reg_id);
      msg = create_sox_message ();

      msg_add_device_installation (msg, event_node, reg_id, "FIREFLY",
                                   "A Firefly Node", time_str);
      // Uptime 
      sprintf (reg_name, "%s_UPTIME", event_node);
      val = reg_id_get (reg_name, reg_id);
      if (val == 0)
        strcpy (reg_id, "unknown");
      msg_add_transducer_installation (msg, event_node, "Uptime", "0001",
                                       reg_id, FALSE);
      sprintf (sensor_raw_str, "%d", stats.uptime);
      sprintf (sensor_adj_str, "%d", stats.uptime);
      msg_add_value_to_transducer (msg, event_node, "0001", sensor_adj_str,
                                   sensor_raw_str, time_str);
      // Deep Sleep 
      sprintf (reg_name, "%s_DEEPSLEEP", event_node);
      val = reg_id_get (reg_name, reg_id);
      if (val == 0)
        strcpy (reg_id, "unknown");
      msg_add_transducer_installation (msg, event_node, "DeepSleep", "0002",
                                       reg_id, FALSE);
      sprintf (sensor_raw_str, "%d", stats.deep_sleep);
      sprintf (sensor_adj_str, "%d", stats.deep_sleep);
      msg_add_value_to_transducer (msg, event_node, "0002", sensor_adj_str,
                                   sensor_raw_str, time_str);

      // Idle Time
      sprintf (reg_name, "%s_IDLETIME", event_node);
      val = reg_id_get (reg_name, reg_id);
      if (val == 0)
        strcpy (reg_id, "unknown");
      msg_add_transducer_installation (msg, event_node, "IdleTime", "0003",
                                       reg_id, FALSE);
      sprintf (sensor_raw_str, "%d", stats.idle_time);
      sprintf (sensor_adj_str, "%d", stats.idle_time);
      msg_add_value_to_transducer (msg, event_node, "0003", sensor_adj_str,
                                   sensor_raw_str, time_str);

      // TX packets 
      sprintf (reg_name, "%s_TXPACKETS", event_node);
      val = reg_id_get (reg_name, reg_id);
      if (val == 0)
        strcpy (reg_id, "unknown");
      msg_add_transducer_installation (msg, event_node, "TxPkts", "0004",
                                       reg_id, FALSE);
      sprintf (sensor_raw_str, "%d", stats.tx_pkts);
      sprintf (sensor_adj_str, "%d", stats.tx_pkts);
      msg_add_value_to_transducer (msg, event_node, "0004", sensor_adj_str,
                                   sensor_raw_str, time_str);

      // RX packets 
      sprintf (reg_name, "%s_RXPACKETS", event_node);
      val = reg_id_get (reg_name, reg_id);
      if (val == 0)
        strcpy (reg_id, "unknown");
      msg_add_transducer_installation (msg, event_node, "RxPkts", "0005",
                                       reg_id, FALSE);
      sprintf (sensor_raw_str, "%d", stats.tx_pkts);
      sprintf (sensor_adj_str, "%d", stats.tx_pkts);
      msg_add_value_to_transducer (msg, event_node, "0005", sensor_adj_str,
                                   sensor_raw_str, time_str);

      // RX failures 
      sprintf (reg_name, "%s_RXFAILURES", event_node);
      val = reg_id_get (reg_name, reg_id);
      if (val == 0)
        strcpy (reg_id, "unknown");
      msg_add_transducer_installation (msg, event_node, "RxFailures", "0006",
                                       reg_id, FALSE);
      sprintf (sensor_raw_str, "%d", stats.rx_failures);
      sprintf (sensor_adj_str, "%d", stats.rx_failures);
      msg_add_value_to_transducer (msg, event_node, "0006", sensor_adj_str,
                                   sensor_raw_str, time_str);

      // TX retries 
      sprintf (reg_name, "%s_TXRETRIES", event_node);
      val = reg_id_get (reg_name, reg_id);
      if (val == 0)
        strcpy (reg_id, "unknown");
      msg_add_transducer_installation (msg, event_node, "TxRetries", "0007",
                                       reg_id, FALSE);
      sprintf (sensor_raw_str, "%d", stats.tx_retry);
      sprintf (sensor_adj_str, "%d", stats.tx_retry);
      msg_add_value_to_transducer (msg, event_node, "0007", sensor_adj_str,
                                   sensor_raw_str, time_str);
      // Sensor Samples 
      sprintf (reg_name, "%s_SENSORSAMPLES", event_node);
      val = reg_id_get (reg_name, reg_id);
      if (val == 0)
        strcpy (reg_id, "unknown");
      msg_add_transducer_installation (msg, event_node, "SensorSamples", "0008",
                                       reg_id, FALSE);
      sprintf (sensor_raw_str, "%d", stats.sensor_samples);
      sprintf (sensor_adj_str, "%d", stats.sensor_samples);
      msg_add_value_to_transducer (msg, event_node, "0008", sensor_adj_str,
                                   sensor_raw_str, time_str);



      ret = publish_sox_message (connection, event_node, msg);
      delete_sox_message (msg);

      if (ret != XMPP_NO_ERROR) {
        sprintf (global_error_msg, "Transducer pkt could not send %s: %s",
                 event_node, ERROR_MESSAGE (ret));
        log_write (global_error_msg);
        return -1;
      } 


		}
	




}

