#include <xml_pkt_parser.h>
#include <stdlib.h>
#include <stdio.h>
#include <expat.h>
#include <error_log.h>
#include <sampl.h>
#include <transducer_pkt.h>
#include <ff_power.h>
#include <transducer_registry.h>
#include <ctype.h>

#define WAIT_STATE		0
#define DS_BUILD_STATE		1
#define P2P_BUILD_STATE		2
#define SLEEP_BUILD_STATE	3

#define MAX_TRANS_MSGS		32

static void XMLCALL startElement (void *data, const char *element_name,
                                  const char **attr);
static void XMLCALL endElement (void *data, const char *element_name);
static int pb_cnt;
static int pb_state;

static TRANSDUCER_PKT_T pb_tran_pkt;
static TRANSDUCER_MSG_T pb_tran_msg;

static uint16_t mac_filter_list[32];
static int mac_filter_size;
static GW_SCRIPT_PKT_T *pb_gw_pkt;
static int *pb_gw_pkt_max;
static SAMPL_DOWNSTREAM_PKT_T pb_ds_pkt;
static SAMPL_PEER_2_PEER_PKT_T pb_p2p_pkt;
static uint8_t pkt_buf[MAX_PAYLOAD];
static uint8_t pb_msg_payload[MAX_PAYLOAD];
static char action[32];
static char params[32];

int build_sampl_pkts_from_xml (GW_SCRIPT_PKT_T * pkts, int max_pkts,
                               char *xml_buf, int size)
{
  char c;
  int i;
  XML_Parser p;

  pb_state = WAIT_STATE;
  pb_cnt = 0;
  pb_gw_pkt = pkts;
  pb_gw_pkt_max = max_pkts;
  //Creates an instance of the XML Parser to parse the event packet
  p = XML_ParserCreate (NULL);
  if (!p) {
    log_write ("Couldnt allocate memory for XML Parser\n");
    return -1;
  }
  //Sets the handlers to call when parsing the start and end of an XML element
  XML_SetElementHandler (p, startElement, endElement);
//  XML_SetCharacterDataHandler (p, charData);

  if (XML_Parse (p, xml_buf, size, 1) == XML_STATUS_ERROR) {
    sprintf (global_error_msg, "XML config file parse error at line %u: %s\n",
             XML_GetCurrentLineNumber (p),
             XML_ErrorString (XML_GetErrorCode (p)));
    log_write (global_error_msg);
    XML_ParserFree (p);
    return -1;
  }

  XML_ParserFree (p);


  return pb_cnt;
}

int load_xml_file (char *filename, char *xml_buf)
{
  FILE *fp;
  char c;
  int v, cnt;

  printf ("opening: %s\n", filename);
  fp = fopen (filename, "r");
  if (fp == NULL) {
    printf ("can not open %s\n", filename);
    return -1;
  }
  cnt = 0;
  do {
    c = fgetc (fp);
    if (cnt < MAX_XML_FILE && c != EOF) {
      xml_buf[cnt] = c;
      cnt++;
    }
  } while (c != EOF);
  xml_buf[cnt] = '\0';
  fclose (fp);
  return cnt;
}



//XMLParser func called whenever the start of an XML element is encountered
static void XMLCALL startElement (void *data, const char *element_name,
                                  const char **attr)
{
  int i;

// Build message destine for node here
  //printf ("element: %s\n", element_name);
  if (strcmp (element_name, "FireFlyDSPacket") == 0) {
    pb_state = DS_BUILD_STATE;
    pb_gw_pkt[pb_cnt].type = DS_PKT;
    pb_ds_pkt.buf = pkt_buf;
    pb_ds_pkt.payload_len = 0;
    pb_ds_pkt.buf_len = DS_PAYLOAD_START;
    pb_ds_pkt.payload_start = DS_PAYLOAD_START;
    pb_ds_pkt.payload = &(pkt_buf[DS_PAYLOAD_START]);
    // Fill in some good default values
    pb_ds_pkt.ctrl_flags = DS_MASK;
    pb_ds_pkt.last_hop_mac = 0;
    pb_ds_pkt.hop_cnt = 0;
    pb_ds_pkt.mac_filter_num = 0;
    pb_ds_pkt.rssi_threshold = -32;
    pb_ds_pkt.hop_max = 5;
    pb_ds_pkt.delay_per_level = 1;
    pb_ds_pkt.mac_check_rate = 100;

    mac_filter_size = 0;
    pb_tran_pkt.num_msgs = 0;
    pb_tran_pkt.checksum = 0;
    pb_tran_pkt.msgs_payload = pkt_buf;
    pb_tran_msg.type= 0;
    pb_tran_msg.payload= pb_msg_payload;
  }
  if (strcmp (element_name, "Sleep") == 0) {
    pb_state = SLEEP_BUILD_STATE;
    pb_gw_pkt[pb_cnt].type = SLEEP;
  }


  for (i = 0; attr[i]; i += 2) {
    const char *attr_name = attr[i];
    const char *attr_value = attr[i + 1];
    //printf ("\tattr_name: %s", attr_name);
    //printf ("\tvalue: %s\n", attr_value);
    switch (pb_state) {
    case SLEEP_BUILD_STATE:
      if (strcmp (attr_name, "value") == 0)
        pb_gw_pkt[pb_cnt].nav = atoi (attr_value);
      break;
    case DS_BUILD_STATE:
      if (strcmp (element_name, "FireFlyDSPacket") == 0) {
        if (strcmp (attr_name, "type") == 0) {
          if (strcmp (attr_value, "ping") == 0)
            pb_ds_pkt.pkt_type = PING_PKT;
          else if (strcmp (attr_value, "transducerCmd") == 0)
            pb_ds_pkt.pkt_type = TRANSDUCER_PKT;
          else if (strcmp (attr_value, "xmppLite") == 0)
            pb_ds_pkt.pkt_type = XMPP_PKT;
          else if (strcmp (attr_value, "control") == 0)
            pb_ds_pkt.pkt_type = CONTROL_PKT;
          else if (strcmp (attr_value, "traceroute") == 0)
            pb_ds_pkt.pkt_type = TRACEROUTE_PKT;
          else if (strcmp (attr_value, "stats") == 0)
            pb_ds_pkt.pkt_type = STATS_PKT;
          else if (strcmp (attr_value, "dataStorage") == 0)
            pb_ds_pkt.pkt_type = DATA_STORAGE_PKT;
        }
        else if (strcmp (attr_name, "nav") == 0) {
          pb_ds_pkt.nav = atoi (attr_value);
          pb_gw_pkt[pb_cnt].nav = pb_ds_pkt.nav;
        }
        else if (strcmp (attr_name, "debugFlag") == 0) {
          if (strcmp (attr_value, "enable") == 0)
            pb_ds_pkt.ctrl_flags |= DEBUG_FLAG;
        }
        else if (strcmp (attr_name, "encryptFlag") == 0) {
          if (strcmp (attr_value, "enable") == 0)
            pb_ds_pkt.ctrl_flags |= ENCRYPT;
        }
        else if (strcmp (attr_name, "treeFilterFlag") == 0) {
          if (strcmp (attr_value, "enable") == 0)
            pb_ds_pkt.ctrl_flags |= TREE_FILTER;
        }
        else if (strcmp (attr_name, "linkAckFlag") == 0) {
          if (strcmp (attr_value, "enable") == 0)
            pb_ds_pkt.ctrl_flags |= LINK_ACK;
        }
        else if (strcmp (attr_name, "prio") == 0) {
          pb_ds_pkt.priority = atoi (attr_value);
        }
        else if (strcmp (attr_name, "seqNum") == 0) {
          pb_ds_pkt.seq_num = atoi (attr_value);
        }
        else if (strcmp (attr_name, "maxHopCnt") == 0) {
          pb_ds_pkt.hop_max = atoi (attr_value);
        }
        else if (strcmp (attr_name, "delayPerLevel") == 0) {
          pb_ds_pkt.delay_per_level = atoi (attr_value);
        }
        else if (strcmp (attr_name, "ackRetry") == 0) {
          pb_ds_pkt.ack_retry = atoi (attr_value);
        }
        else if (strcmp (attr_name, "MACCheckRate") == 0) {
          pb_ds_pkt.mac_check_rate = atoi (attr_value);
        }
        else if (strcmp (attr_name, "rssiThresh") == 0) {
          pb_ds_pkt.rssi_threshold = atoi (attr_value);
        }


      }
      else if (strcmp (element_name, "MACFilter") == 0) {
        int tmp;
        sscanf (attr_value, "%x", &tmp);
        downstream_packet_add_mac_filter (&pb_ds_pkt, tmp);
      }
      else if (strcmp (element_name, "Transducer") == 0) {
        int tmp;
        if (strcmp (attr_name, "macAddr") == 0) {
          sscanf (attr_value, "%x", &tmp);
          pb_tran_msg.mac_addr = tmp;
        }
        if (strcmp (attr_name, "type") == 0) {
          // These are common values copied from transducer_registry.h
          if (strcmp (attr_value, "TRAN_FF_BASIC_SHORT") == 0) {
            pb_tran_msg.type= TRAN_FF_BASIC_SHORT;
          }
          else if (strcmp (attr_value, "TRAN_LED_BLINK") == 0)
            pb_tran_msg.type= TRAN_LED_BLINK;
          else if (strcmp (attr_value, "TRAN_BINARY_SENSOR") == 0)
            pb_tran_msg.type= TRAN_BINARY_SENSOR;
          else if (strcmp (attr_value, "TRAN_POWER_PKT") == 0)
            pb_tran_msg.type= TRAN_POWER_PKT;
          else {
            sscanf (attr_value, "%x", &tmp);
            pb_tran_msg.type= tmp;
          }
        }
        if (strcmp (attr_name, "action") == 0) {
		sprintf( action,"%s",attr_value );
          }
        if (strcmp (attr_name, "params") == 0) {
		sprintf( params,"%s",attr_value );
          }


        }

      break;

    }

  }

}

//XMLParser func called whenever an end of element is encountered
static void XMLCALL endElement (void *data, const char *element_name)
{
  int i;
  FF_POWER_RQST_PKT	ff_pwr_rqst;
  FF_POWER_ACTUATE_PKT	ff_pwr_actuate;
  //printf( "end=%s\n",element_name );
  if (strcmp (element_name, "FireFlyDSPacket") == 0) {
    pb_state = WAIT_STATE;
    if (pb_tran_pkt.num_msgs > 0) {
	// Pack tran_pkt
	pb_ds_pkt.payload_len =
		transducer_pkt_pack(&pb_tran_pkt,pb_ds_pkt.payload);
    }
    pack_downstream_packet (&pb_ds_pkt);
    pb_gw_pkt[pb_cnt].size = pb_ds_pkt.buf_len;
    for (i = 0; i < pb_ds_pkt.buf_len; i++)
      pb_gw_pkt[pb_cnt].pkt[i] = pb_ds_pkt.buf[i];
    pb_cnt++;
  }
  else if (strcmp (element_name, "Transducer") == 0)
	{
	// Add msg to tran_pkt
	switch(pb_tran_msg.type)
		{
		case TRAN_FF_BASIC_SHORT:
		    // don't need to do anything really...	
		    pb_tran_msg.len=0;	
		    //transducer_msg_add( &pb_tran_pkt, &pb_tran_msg );	
		break;
		case TRAN_LED_BLINK:
		    // don't need to do anything really...	
		    if(strstr(params,"RED")!=0) pb_tran_msg.payload[0]|=TRAN_RED_LED_MASK;	
		    if(strstr(params,"GREEN")!=0) pb_tran_msg.payload[0]|=TRAN_GREEN_LED_MASK;	
		    if(strstr(params,"BLUE")!=0) pb_tran_msg.payload[0]|=TRAN_BLUE_LED_MASK;	
		    if(strstr(params,"ORANGE")!=0) pb_tran_msg.payload[0]|=TRAN_ORANGE_LED_MASK;	
		    pb_tran_msg.len=1;	
		break;
		case TRAN_POWER_PKT:
			if(strcmp(action,"sense")==0 )
			{
				if(strstr(params,"1")!=0) ff_pwr_rqst.socket=1;
				else ff_pwr_rqst.socket=0;
        			ff_pwr_rqst.pkt_type=SENSE_PKT;
				pb_tran_msg.len=ff_power_rqst_pack(pb_tran_msg.payload, &ff_pwr_rqst);
				printf( "Transducer sense packet!\n" );
			}
			if(strcmp(action,"actuate")==0 )
			{
				ff_pwr_actuate.socket0_state=SOCKET_HOLD;
				ff_pwr_actuate.socket1_state=SOCKET_HOLD;
				if(strstr(params,"0")!=0) 
				{
					if(strstr(params,"on")!=0) ff_pwr_actuate.socket0_state=SOCKET_ON;
					if(strstr(params,"off")!=0) ff_pwr_actuate.socket0_state=SOCKET_OFF;
				}
        			else if(strstr(params,"1")!=0) 
				{
					if(strstr(params,"on")!=0) ff_pwr_actuate.socket1_state=SOCKET_ON;
					if(strstr(params,"off")!=0) ff_pwr_actuate.socket1_state=SOCKET_OFF;
				}
				ff_pwr_actuate.type=ACTUATE_PKT;
				pb_tran_msg.len=ff_power_actuate_pack(pb_tran_msg.payload, &ff_pwr_actuate);
			}

		break;
		}
	transducer_msg_add( &pb_tran_pkt, &pb_tran_msg );	
	}
  else if (strcmp (element_name, "Sleep") == 0) {
    pb_state = WAIT_STATE;
    pb_cnt++;
  }

}
