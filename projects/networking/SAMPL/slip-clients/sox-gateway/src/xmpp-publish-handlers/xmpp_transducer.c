#include <stdlib.h>
#include <time.h>
#include <xmpp_transducer.h>
#include <sampl.h>
#include <soxlib.h>
#include <node_cache.h>
#include <globals.h>
#include <xmpp_pkt.h>
#include <ack_pkt.h>
#include <ff_basic_sensor_pkt.h>
#include <ff_basic_sensor_cal.h>
#include <jiga_watt_cal.h>
#include <nlist_tran.h>
#include <transducer_pkt.h>
#include <xmpp_transducer.h>
#include <transducer_registry.h>
#include <ff_power.h>
#include <xmpp_proxy.h>
#include <tx_queue.h>
#include <error_log.h>


#define TEMPERATURE_OFFSET 400


static uint8_t freq;
static uint16_t adc_rms_current, adc_rms_voltage;
static uint16_t adc_rms_current2;
static uint32_t adc_true_power, adc_energy, time_secs;
static uint32_t adc_true_power2, adc_energy2;
static uint16_t adc_v_p2p_low, adc_v_p2p_high;
static uint16_t adc_i_p2p_low, adc_i_p2p_high;
static uint16_t adc_i_p2p_low2, adc_i_p2p_high2;
static float cal_energy;
static long long tmp_energy;

static float rms_voltage, rms_current, true_power, apparent_power,
  power_factor;
static float rms_current2, true_power2, apparent_power2, power_factor2;

static jiga_watt_cal_struct_t jiga_watt_cal;

void publish_xmpp_transducer_pkt (SAMPL_GATEWAY_PKT_T * gw_pkt)
{
  uint8_t i;
  int8_t v;
  XMPP_POWER_T xmpp_pwr;
  FF_SENSOR_SHORT_PKT_T sensor_short;
  ACK_PKT_T ack;
  NLIST_TRAN_PKT   nlist;
  TRANSDUCER_PKT_T tp;
  TRANSDUCER_MSG_T tm;
  FF_POWER_SENSE_PKT ps;
  FF_POWER_DEBUG_PKT pd;
  SOXMessage *msg = NULL;
  char sensor_raw_str[32];
  char sensor_adj_str[32];
  char sensor_type_str[32];
  char event_node[32];
  char reg_name[32];
  char rssiStr[5];
  char neighbor_name[32];
  char time_str[100];
  char data_str[255];
  char reg_id[100];
  int ret = 0, val,j;
  float temp_cal, battery_cal;
  uint16_t light_cal;
  uint32_t c_mac;
  time_t timestamp;

		  uint8_t tran_ack;


  if (gw_pkt->pkt_type != TRANSDUCER_PKT)
    return 0;
  // GW reply packets do not include the tran_pkt header
  // since this information can be captured in the gw
  // packet header info.  So fill out a dummy packet
  v = transducer_pkt_unpack (&tp, gw_pkt->payload);
  if (v != 1) {
    printf ("Transducer checksum failed!\n");
  }
  for (i = 0; i < tp.num_msgs; i++) {
    v = transducer_msg_get (&tp, &tm, i);
    switch (tm.type) {

	case TRAN_BINARY_SENSOR:
		{
		uint8_t binary_sensor_type, binary_sensor_value;
 
		time (&timestamp); strftime (time_str, 100, "%Y-%m-%dT%X", localtime (&timestamp));
        printf ("Binary Sensor from 0x%x\n", tm.mac_addr);
        binary_sensor_type=tm.payload[0]&0x7f;
        binary_sensor_value=tm.payload[0]>>7;
			printf( "     binary sensor type: %d value: %d\n", binary_sensor_type, binary_sensor_value ); 

      c_mac =
        gw_pkt->subnet_mac[2] << 24 | gw_pkt->subnet_mac[1] << 16 | gw_pkt->
        subnet_mac[0] << 8 | tm.mac_addr;
  
	sprintf (event_node, "%02x%02x%02x%02x", gw_pkt->subnet_mac[2],
                 gw_pkt->subnet_mac[1],
                 gw_pkt->subnet_mac[0], tm.mac_addr);


        check_and_create_node (event_node);
        val = reg_id_get (event_node, reg_id);
        if (val == 0)
          strcpy (reg_id, "unknown");
        msg = create_sox_message ();
        msg_add_device_installation (msg, event_node, reg_id, "FIREFLY",
                                     "A Firefly Node", time_str);


			if(binary_sensor_type==BINARY_SENSOR_MOTION) sprintf (sensor_type_str, "Motion");
			if(binary_sensor_type==BINARY_SENSOR_GPIO) sprintf (sensor_type_str, "GPIO");
			if(binary_sensor_type==BINARY_SENSOR_LEAK) sprintf (sensor_type_str, "Water");
			if(binary_sensor_value==1) strcpy(sensor_raw_str,"true"); else strcpy(sensor_raw_str,"false");
        sprintf (sensor_adj_str, "%d", binary_sensor_value);

        msg_add_transducer_installation (msg, event_node, sensor_type_str, "0001", reg_id, FALSE);
        msg_add_value_to_transducer (msg, event_node, "0001", sensor_adj_str, sensor_raw_str, time_str);

		if(xmpp_flag)
		{
        ret = publish_sox_message (connection, event_node, msg);
        delete_sox_message (msg);
        if (ret != XMPP_NO_ERROR) {
          sprintf (global_error_msg, "Could not publish Jiga Watt to %s: %s",
                   event_node, ERROR_MESSAGE (ret));
          log_write (global_error_msg);
          return -1;
        }
	}
	}

	break;


    case TRAN_BINARY_BLOB:
        c_mac =
        gw_pkt->subnet_mac[2] << 24 | gw_pkt->subnet_mac[1] << 16 | gw_pkt->
        subnet_mac[0] << 8 | tm.mac_addr;
	printf( "  from: 0x%08x\n",c_mac);
	printf( "  size: %u\n  0x",tm.len );
	data_str[0]='\0';
	for(j=0; j<tm.len; j++ ) { 
		sprintf( data_str,"%s%02x",data_str,tm.payload[j] );
	}
	printf( "\n" );
	sprintf( event_node,"%08x",c_mac ); 
        check_and_create_node (event_node);
  	time (&timestamp);
  	strftime (time_str, 100, "%Y-%m-%d %X", localtime (&timestamp));

  	msg = create_sox_message();
  	msg_add_device_installation(msg,event_node,event_node,"FIREFLY","A Firefly Node",time_str);

	if(debug_txt_flag) printf( "Binary Blob from: %s\n",event_node );
	
        msg_add_transducer_installation (msg, event_node, "HexData", "0001",
                                       "unknown", FALSE);

        msg_add_value_to_transducer (msg, event_node, "0001", "n/a",
                                   data_str, time_str);
  	if(debug_txt_flag) printf( "  hex data: %s\n",data_str);


  	ret = publish_sox_message (connection, event_node, msg);
  	delete_sox_message(msg);
  	if (ret != XMPP_NO_ERROR) {
    		sprintf (global_error_msg, "XMPP Error Extended nList: %s", ERROR_MESSAGE (ret));
    		log_write (global_error_msg);
    		sprintf (global_error_msg, "-> event_node=%s",event_node);
    		log_write (global_error_msg);
    		return -1;
  	}



	break;

    case TRAN_NLIST:
        nlist_tran_unpack (&tm, &nlist);
	if(debug_txt_flag) printf( "Neighbor List Transducer Pkt!\n" );
        c_mac =
        gw_pkt->subnet_mac[2] << 24 | gw_pkt->subnet_mac[1] << 16 | gw_pkt->
        subnet_mac[0] << 8 | tm.mac_addr;
	sprintf( event_node,"%08x",c_mac ); 
        check_and_create_node (event_node);
  	time (&timestamp);
  	strftime (time_str, 100, "%Y-%m-%d %X", localtime (&timestamp));

  	msg = create_sox_message();
  	msg_add_device_installation(msg,event_node,event_node,"FIREFLY","A Firefly Node",time_str);

	if(debug_txt_flag) printf( "%s Neighbors:\n",event_node );
	for(j=0; j<nlist.num_neighbors; j++ )
	{
	uint32_t nmac;
    	sprintf( rssiStr,"%d",nlist.neighbor[j].rssi );
	nmac= nlist.neighbor[j].mac[3]<<24 | nlist.neighbor[j].mac[2]<<16 | nlist.neighbor[j].mac[1]<<8 | nlist.neighbor[j].mac[0];
    	sprintf(neighbor_name,"%08x",nmac );
	msg_add_device_connection(msg,event_node,"unknown",neighbor_name,rssiStr);
  	if(debug_txt_flag) printf( "  node: %s rssi: %s\n",neighbor_name,rssiStr );
  	}


  	ret = publish_sox_message (connection, event_node, msg);
  	delete_sox_message(msg);
  	if (ret != XMPP_NO_ERROR) {
    		sprintf (global_error_msg, "XMPP Error Extended nList: %s", ERROR_MESSAGE (ret));
    		log_write (global_error_msg);
    		sprintf (global_error_msg, "-> event_node=%s",event_node);
    		log_write (global_error_msg);
    		return -1;
  	}

	break;
    case TRAN_FF_BASIC_SHORT:
      ff_basic_sensor_short_unpack (&tm, &sensor_short);
      printf ("Sensor pkt from 0x%x\n", tm.mac_addr);
      printf ("   Light: %d\n", sensor_short.light);
      printf ("   Temperature: %d\n", sensor_short.temperature);
      printf ("   Acceleration: %d\n", sensor_short.acceleration);
      printf ("   Sound Level: %d\n", sensor_short.sound_level);
      printf ("   Battery: %d\n", sensor_short.battery + 100);

      c_mac =
        gw_pkt->subnet_mac[2] << 24 | gw_pkt->subnet_mac[1] << 16 | gw_pkt->
        subnet_mac[0] << 8 | tm.mac_addr;
      // calibrate temperature
      temp_cal =
        ff_basic_sensor_cal_get_temp ((sensor_short.temperature << 1) +
                                      TEMPERATURE_OFFSET) +
        ff_basic_sensor_cal_get_temp_offset (c_mac);
      // calibrate light
      printf ("battery=%u light=%u\n", sensor_short.battery,
              sensor_short.light);
      light_cal = ff_basic_sensor_cal_get_light (sensor_short.light, ((float)
        (sensor_short.battery + 100)) / 100) + ff_basic_sensor_cal_get_light_offset (c_mac);
      light_cal = 255 - light_cal;
      battery_cal = ((float) (sensor_short.battery + 100)) / 100;
      // Get local timestamp
      time (&timestamp);
      strftime (time_str, 100, "%Y-%m-%dT%X", localtime (&timestamp));
      printf ("Sensor pkt from 0x%x\n", tm.mac_addr);
      printf ("   Light: %u (%u)\n", light_cal, sensor_short.light);
      printf ("   Temperature: %f (%u)\n", temp_cal,
              sensor_short.temperature);
      printf ("   Acceleration: %u\n", sensor_short.acceleration);
      printf ("   Sound Level: %u\n", sensor_short.sound_level);
      printf ("   Battery: %f (%u)\n", battery_cal, sensor_short.battery);
      printf ("   Timestamp: %s\n", time_str);

      sprintf (event_node, "%02x%02x%02x%02x", gw_pkt->subnet_mac[2],
               gw_pkt->subnet_mac[1], gw_pkt->subnet_mac[0], tm.mac_addr);


      printf ("event node=%s\n", event_node);


      check_and_create_node (event_node);
      val = reg_id_get (event_node, reg_id);
      if (val == 0)
        strcpy (reg_id, "unknown");
      printf ("    event node: %s reg_id: %s\n", event_node, reg_id);
      msg = create_sox_message ();

      msg_add_device_installation (msg, event_node, reg_id, "FIREFLY",
                                   "A Firefly Node", time_str);
      // Light
      sprintf (reg_name, "%s_LIGHT", event_node);
      val = reg_id_get (reg_name, reg_id);
      if (val == 0)
        strcpy (reg_id, "unknown");
      msg_add_transducer_installation (msg, event_node, "Light", "0001",
                                       reg_id, FALSE);
      sprintf (sensor_raw_str, "%d", sensor_short.light);
      sprintf (sensor_adj_str, "%d", light_cal);
      msg_add_value_to_transducer (msg, event_node, "0001", sensor_adj_str,
                                   sensor_raw_str, time_str);
      // Temperature
      sprintf (reg_name, "%s_TEMP", event_node);
      val = reg_id_get (reg_name, reg_id);
      if (val == 0)
        strcpy (reg_id, "unknown");
      msg_add_transducer_installation (msg, event_node, "Temperature",
                                       "0002", reg_id, FALSE);
      sprintf (sensor_raw_str, "%d", sensor_short.light);
      sprintf (sensor_adj_str, "%2.2f", temp_cal);
      msg_add_value_to_transducer (msg, event_node, "0002", sensor_adj_str,
                                   sensor_raw_str, time_str);
      // Acceleration 
      sprintf (reg_name, "%s_ACCEL", event_node);
      val = reg_id_get (reg_name, reg_id);
      if (val == 0)
        strcpy (reg_id, "unknown");
      msg_add_transducer_installation (msg, event_node, "Acceleration",
                                       "0003", reg_id, FALSE);
      sprintf (sensor_raw_str, "%d", sensor_short.acceleration);
      sprintf (sensor_adj_str, "%d", sensor_short.acceleration);
      msg_add_value_to_transducer (msg, event_node, "0003", sensor_adj_str,
                                   sensor_raw_str, time_str);
      // Voltage 
      sprintf (reg_name, "%s_BAT", event_node);
      val = reg_id_get (reg_name, reg_id);
      if (val == 0)
        strcpy (reg_id, "unknown");
      msg_add_transducer_installation (msg, event_node, "Voltage", "0004",
                                       reg_id, FALSE);
      sprintf (sensor_raw_str, "%d", sensor_short.battery);
      sprintf (sensor_adj_str, "%1.2f", battery_cal);
      msg_add_value_to_transducer (msg, event_node, "0004", sensor_adj_str,
                                   sensor_raw_str, time_str);
      // Sound Level 
      sprintf (reg_name, "%s_AUDIO", event_node);
      val = reg_id_get (reg_name, reg_id);
      if (val == 0)
        strcpy (reg_id, "unknown");
      msg_add_transducer_installation (msg, event_node, "Audio", "0005",
                                       reg_id, FALSE);
      sprintf (sensor_raw_str, "%d", sensor_short.sound_level);
      sprintf (sensor_adj_str, "%d", sensor_short.sound_level);
      msg_add_value_to_transducer (msg, event_node, "0005", sensor_adj_str,
                                   sensor_raw_str, time_str);

      ret = publish_sox_message (connection, event_node, msg);
      delete_sox_message (msg);

      if (ret != XMPP_NO_ERROR) {
        sprintf (global_error_msg, "Transducer pkt could not send %s: %s",
                 event_node, ERROR_MESSAGE (ret));
        log_write (global_error_msg);
        return -1;
      }

      break;


    case TRAN_NCK:
    case TRAN_ACK:

		  if(tm.type==TRAN_ACK) tran_ack=1;
			else tran_ack=0;
      if(debug_txt_flag==1)
      printf ("Transducer ACK from 0x%x with value %d\n", tm.mac_addr,tran_ack);
      
        		sprintf (event_node, "%02x%02x%02x%02x", gw_pkt->subnet_mac[2],
                 		gw_pkt->subnet_mac[1],
                 		gw_pkt->subnet_mac[0], tm.mac_addr);
			check_and_create_node (event_node);

        time (&timestamp);
        strftime (time_str, 100, "%Y-%m-%dT%X", localtime (&timestamp));

			  val = reg_id_get (event_node, reg_id);
        if (val == 0)
          strcpy (reg_id, "unknown");
        msg = create_sox_message ();
        msg_add_device_installation (msg, event_node, reg_id, "JigaWatt",
                                     "A Firefly Node with Power", time_str);

	
        sprintf (reg_name, "%s_TRANACK", event_node);
        val = reg_id_get (reg_name, reg_id);
        if (val == 0)
          strcpy (reg_id, "unknown");
        sprintf (sensor_type_str, "TranACK" );
        msg_add_transducer_installation (msg, event_node, sensor_type_str, "0001", reg_id, FALSE);
        if(tran_ack==0)
				{
					sprintf (sensor_raw_str, "0");
					sprintf (sensor_adj_str, "0");
				}
        if(tran_ack==1)
				{
					sprintf (sensor_raw_str, "1");
					sprintf (sensor_adj_str, "1");
				}

         msg_add_value_to_transducer (msg, event_node, "0001", sensor_adj_str, sensor_raw_str, time_str);


	if(xmpp_flag)
	{
        ret = publish_sox_message (connection, event_node, msg);
        delete_sox_message (msg);
        if (ret != XMPP_NO_ERROR) {
          sprintf (global_error_msg, "Could not publish Tran ACK data to %s: %s",
                   event_node, ERROR_MESSAGE (ret));
          log_write (global_error_msg);
          return -1;
        }
	} else delete_sox_message (msg);

			
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

        		sprintf (event_node, "%02x%02x%02x%02x", gw_pkt->subnet_mac[2],
                 		gw_pkt->subnet_mac[1],
                 		gw_pkt->subnet_mac[0], tm.mac_addr);

			xmpp_pwr.socket=ps.socket_num;
			xmpp_pwr.adc_rms_current=ps.rms_current;
			xmpp_pwr.adc_rms_voltage=ps.rms_voltage;
			xmpp_pwr.adc_true_power=ps.true_power;
			xmpp_pwr.adc_energy=tmp_energy;
			xmpp_pwr.rms_current=rms_current;
			xmpp_pwr.rms_voltage=rms_voltage;
			xmpp_pwr.apparent_power=rms_current*rms_voltage;
			xmpp_pwr.true_power=true_power;
			xmpp_pwr.energy=cal_energy;
			if( ps.socket_num==0) xmpp_pwr.state=ps.socket0_state;
			if( ps.socket_num==1) xmpp_pwr.state=ps.socket1_state;
			if(apparent_power==0)
				xmpp_pwr.power_factor=0;
			else
				xmpp_pwr.power_factor=apparent_power/true_power;
			if(xmpp_pwr.power_factor>1.0) xmpp_pwr.power_factor=1.0;
			xmpp_pwr.freq=60; // FIXME: do we get freq back?
			val=send_xmpp_power(event_node, xmpp_pwr  );
			if(val==-1) return -1;

       } else
              if(tm.payload[0]==DEBUG_PKT)
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

        		sprintf (event_node, "%02x%02x%02x%02x", gw_pkt->subnet_mac[2],
                 		gw_pkt->subnet_mac[1],
                 		gw_pkt->subnet_mac[0], tm.mac_addr);

			xmpp_pwr.socket=0;
			xmpp_pwr.adc_rms_current=pd.rms_current;
			xmpp_pwr.adc_rms_voltage=pd.rms_voltage;
			xmpp_pwr.adc_true_power=pd.true_power;
			xmpp_pwr.adc_energy=tmp_energy;
			xmpp_pwr.rms_current=rms_current;
			xmpp_pwr.rms_voltage=rms_voltage;
			xmpp_pwr.apparent_power=rms_current*rms_voltage;
			xmpp_pwr.true_power=true_power;
			xmpp_pwr.energy=cal_energy;
			xmpp_pwr.state=pd.socket0_state;
			if(apparent_power==0)
				xmpp_pwr.power_factor=0;
			else
				xmpp_pwr.power_factor=apparent_power/true_power;
			if(xmpp_pwr.power_factor>1.0) xmpp_pwr.power_factor=1.0;
			xmpp_pwr.freq=pd.freq;
			val=send_xmpp_power(event_node, xmpp_pwr  );
			if(val==-1) return -1;


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


			xmpp_pwr.socket=1;
			xmpp_pwr.adc_rms_current=pd.rms_current2;
			xmpp_pwr.adc_rms_voltage=pd.rms_voltage;
			xmpp_pwr.adc_true_power=pd.true_power2;
			xmpp_pwr.adc_energy=tmp_energy;
			xmpp_pwr.rms_current=rms_current;
			xmpp_pwr.rms_voltage=rms_voltage;
			xmpp_pwr.apparent_power=rms_current*rms_voltage;
			xmpp_pwr.true_power=true_power;
			xmpp_pwr.energy=cal_energy;
			xmpp_pwr.state=pd.socket1_state;
			if(apparent_power==0)
				xmpp_pwr.power_factor=0;
			else
				xmpp_pwr.power_factor=apparent_power/true_power;
			if(xmpp_pwr.power_factor>1.0) xmpp_pwr.power_factor=1.0;
			xmpp_pwr.freq=pd.freq;
			val=send_xmpp_power(event_node, xmpp_pwr  );
			if(val==-1) return -1;

               }

                break;



    default:
      printf ("unkown transducer packet: %d\n", tm.type);
    }
  }


}

int send_xmpp_power(char *event_node, XMPP_POWER_T pwr  )
{
  char sensor_raw_str[32];
  char sensor_adj_str[32];
  char sensor_type_str[32];
  char reg_name[32];
  char time_str[100];
  char reg_id[100];
  int ret = 0, val;
  uint32_t c_mac;
  SOXMessage *msg = NULL;
  time_t timestamp;
	time (&timestamp);
  strftime (time_str, 100, "%Y-%m-%dT%X", localtime (&timestamp));
 
        check_and_create_node (event_node);
        val = reg_id_get (event_node, reg_id);
        if (val == 0)
          strcpy (reg_id, "unknown");
        msg = create_sox_message ();
        msg_add_device_installation (msg, event_node, reg_id, "JigaWatt",
                                     "A Firefly Node with Power", time_str);

	
        sprintf (reg_name, "%s_VOLTAGE_%d", event_node,pwr.socket);
        val = reg_id_get (reg_name, reg_id);
        if (val == 0)
          strcpy (reg_id, "unknown");
        sprintf (sensor_type_str, "Voltage%d", pwr.socket);
        msg_add_transducer_installation (msg, event_node, sensor_type_str, "0001", reg_id, FALSE);
        sprintf (sensor_raw_str, "%d", pwr.adc_rms_voltage);
        sprintf (sensor_adj_str, "%f", pwr.rms_voltage);
        msg_add_value_to_transducer (msg, event_node, "0001", sensor_adj_str, sensor_raw_str, time_str);

        sprintf (reg_name, "%s_CURRENT_%d", event_node,pwr.socket);
        val = reg_id_get (reg_name, reg_id);
        if (val == 0)
          strcpy (reg_id, "unknown");
        sprintf (sensor_type_str, "Current%d", pwr.socket);
        msg_add_transducer_installation (msg, event_node, sensor_type_str, "0002", reg_id, FALSE);
        sprintf (sensor_raw_str, "%d", pwr.adc_rms_current);
        sprintf (sensor_adj_str, "%f", pwr.rms_current);
        msg_add_value_to_transducer (msg, event_node, "0002", sensor_adj_str, sensor_raw_str, time_str);

        sprintf (reg_name, "%s_POWER_%d", event_node,pwr.socket );
        val = reg_id_get (reg_name, reg_id);
        if (val == 0)
          strcpy (reg_id, "unknown");
        sprintf (sensor_type_str, "TruePower%d", pwr.socket);
        msg_add_transducer_installation (msg, event_node, sensor_type_str, "0003", reg_id, FALSE);
        sprintf (sensor_raw_str, "%d", pwr.adc_true_power);
        sprintf (sensor_adj_str, "%f", true_power);
        msg_add_value_to_transducer (msg, event_node, "0003", sensor_adj_str, sensor_raw_str, time_str);

        sprintf (reg_name, "%s_APOWER_%d", event_node,pwr.socket); 
	val = reg_id_get (reg_name, reg_id); if (val == 0) strcpy (reg_id, "unknown");
        sprintf (sensor_type_str, "ApparentPower%d", pwr.socket);
        msg_add_transducer_installation (msg, event_node, sensor_type_str, "0004", reg_id, FALSE);
        sprintf (sensor_raw_str, "%f", pwr.apparent_power);
        sprintf (sensor_adj_str, "%f", pwr.apparent_power);
        msg_add_value_to_transducer (msg, event_node, "0004", sensor_adj_str, sensor_raw_str, time_str);

        sprintf (reg_name, "%s_POWER_FACTOR_%d", event_node,pwr.socket); 
	val = reg_id_get (reg_name, reg_id); if (val == 0) strcpy (reg_id, "unknown");
        sprintf (sensor_type_str, "PowerFactor%d", pwr.socket);
        msg_add_transducer_installation (msg, event_node, sensor_type_str, "0005", reg_id, FALSE);
        sprintf (sensor_raw_str, "%f", pwr.power_factor);
        sprintf (sensor_adj_str, "%f", pwr.power_factor);
        msg_add_value_to_transducer (msg, event_node, "0005", sensor_adj_str, sensor_raw_str, time_str);

        sprintf (reg_name, "%s_ENERGY_%d", event_node,pwr.socket); 
	val = reg_id_get (reg_name, reg_id); if (val == 0) strcpy (reg_id, "unknown");
        sprintf (sensor_type_str, "Energy%d", pwr.socket);
        msg_add_transducer_installation (msg, event_node, sensor_type_str, "0006", reg_id, FALSE);
        sprintf (sensor_raw_str, "%d", pwr.adc_energy);
        sprintf (sensor_adj_str, "%f", pwr.energy);
        msg_add_value_to_transducer (msg, event_node, "0006", sensor_adj_str, sensor_raw_str, time_str);

        sprintf (reg_name, "%s_STATE_%d", event_node,pwr.socket); 
	val = reg_id_get (reg_name, reg_id); if (val == 0) strcpy (reg_id, "unknown");
        sprintf (sensor_type_str, "State%d", pwr.socket);
        msg_add_transducer_installation (msg, event_node, sensor_type_str, "0007", reg_id, FALSE);
        sprintf (sensor_raw_str, "%d", pwr.state);
        sprintf (sensor_adj_str, "%d", pwr.state);
        msg_add_value_to_transducer (msg, event_node, "0007", sensor_adj_str, sensor_raw_str, time_str);

        sprintf (reg_name, "%s_FREQ_%d", event_node,pwr.socket); 
	val = reg_id_get (reg_name, reg_id); if (val == 0) strcpy (reg_id, "unknown");
        sprintf (sensor_type_str, "Freq%d", pwr.socket);
        msg_add_transducer_installation (msg, event_node, sensor_type_str, "0008", reg_id, FALSE);
        sprintf (sensor_raw_str, "%d", pwr.freq);
        sprintf (sensor_adj_str, "%d", pwr.freq);
        msg_add_value_to_transducer (msg, event_node, "0008", sensor_adj_str, sensor_raw_str, time_str);



	if(xmpp_flag)
	{
        ret = publish_sox_message (connection, event_node, msg);
        delete_sox_message (msg);
        if (ret != XMPP_NO_ERROR) {
          sprintf (global_error_msg, "Could not publish JigaWatt data to %s: %s",
                   event_node, ERROR_MESSAGE (ret));
          log_write (global_error_msg);
          return -1;
        }
	} else delete_sox_message (msg);



return 1;
}
/*  FF_SENSOR_SHORT_PKT_T sensor_short;
  ACK_PKT_T ack;
  TRANSDUCER_REPLY_PKT_T tran_pkt;
  int i, val;
  uint8_t t;
  SOXMessage *msg = NULL;
  char sensor_raw_str[32];
  char sensor_adj_str[32];
  char sensor_type_str[32];
  char event_node[32];
  char reg_name[32];
  char time_str[100];
  char reg_id[100];
  int ret = 0;
  float temp_cal, battery_cal;
  uint16_t light_cal;
  uint32_t c_mac;
  time_t timestamp;

  //      unpack_gateway_packet(&gw_pkt );
  // You will have a gateway packet here to operate on.
  // The gateway packet has a payload which contains user defined packets.

  // Lets print the raw packet:
  //print_gw_packet(&gw_pkt);


case TRAN_BINARY_SENSOR:
	{
	uint8_t binary_sensor_type, binary_sensor_value;
 
	time (&timestamp);
        strftime (time_str, 100, "%Y-%m-%dT%X", localtime (&timestamp));
        printf ("Binary Sensor from 0x%x\n", tran_pkt.mac_addr);
        binary_sensor_type=tran_pkt.payload[0]&0x7f;
        binary_sensor_value=tran_pkt.payload[0]>>7;
	printf( "     binary sensor type: %d value: %d\n", binary_sensor_type, binary_sensor_value ); 

        sprintf (event_node, "%02x%02x%02x%02x", gw_pkt->subnet_mac[2],
                 gw_pkt->subnet_mac[1],
                 gw_pkt->subnet_mac[0], tran_pkt.mac_addr);


        check_and_create_node (event_node);
        val = reg_id_get (event_node, reg_id);
        if (val == 0)
          strcpy (reg_id, "unknown");
        msg = create_sox_message ();
        msg_add_device_installation (msg, event_node, reg_id, "FIREFLY",
                                     "A Firefly Node", time_str);


	if(binary_sensor_type==BINARY_SENSOR_MOTION) sprintf (sensor_type_str, "Motion");
	if(binary_sensor_type==BINARY_SENSOR_GPIO) sprintf (sensor_type_str, "GPIO");
	if(binary_sensor_type==BINARY_SENSOR_LEAK) sprintf (sensor_type_str, "Water");
	if(binary_sensor_value==1) strcpy(sensor_raw_str,"true"); else strcpy(sensor_raw_str,"false");
        sprintf (sensor_adj_str, "%d", binary_sensor_value);

        msg_add_transducer_installation (msg, event_node, sensor_type_str, "0001", reg_id, FALSE);
        msg_add_value_to_transducer (msg, event_node, "0001", sensor_adj_str, sensor_raw_str, time_str);

	if(xmpp_flag)
	{
        ret = publish_sox_message (connection, event_node, msg);
        delete_sox_message (msg);
        if (ret != XMPP_NO_ERROR) {
          sprintf (global_error_msg, "Could not publish Jiga Watt to %s: %s",
                   event_node, ERROR_MESSAGE (ret));
          log_write (global_error_msg);
          return -1;
        }
	}
	}

	break;

*/
