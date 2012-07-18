#include <xmpp_proxy.h>
#include <loudmouth/loudmouth.h>
#include <expat.h>
#include <soxlib.h>
#include <time.h>
#include <sys/types.h>
#include <stdlib.h>
#include <stdio.h>
#include <xmpp_pkt.h>
#include <sampl.h>
#include <globals.h>
#include <error_log.h>


static uint8_t in_body;

static uint8_t proxy_msg_buf[128];
static uint8_t proxy_msg_size;

static uint8_t seq_num;
static uint8_t tmp_src_jid[128];
static uint8_t tmp_dst_jid[128];
static uint8_t tmp_msg[100];
static uint8_t tmp_pass[32];
static XMPP_PKT_T xmpp_pkt_in;

static uint32_t p_port;
static uint32_t last_time;
static uint8_t msg_buf[1024];

void proxy_configure (char *xmpp_server, uint32_t xmpp_server_port,
                      char *pubsub_server, char *xmpp_ssl_fingerprint)
{
  int i;
  seq_num = 0;
  strcpy (p_server, xmpp_server);
  strcpy (p_pubsub, pubsub_server);
  strcpy (p_ssl_fingerprint, xmpp_ssl_fingerprint);
  p_port = xmpp_server_port;
  for (i = 0; i < MAX_PROXY_CONS; i++)
    proxy_con[i].active = 0;
  last_time = 0;
}

uint32_t proxy_find_mac_addr (char *jid)
{
  int i;

  if (jid == NULL)
    return 0;
  for (i = 0; i < MAX_PROXY_CONS; i++) {
    if (proxy_con[i].active == 1 && strcmp (jid, proxy_con[i].src_jid) == 0)
      return proxy_con[i].mac_addr;
  }
  return 0;
}


void proxy_cleanup ()
{
  int i;
  uint32_t sub_time;

// Check if this is the first time proxy_cleanup has been called.
// If so, then set last_time and exit.  We will start cleaning on
// the next calls.
  if (last_time == 0) {
    last_time = time (NULL);
    return;
  }
  else {
    // Figure out how much time has elapsed since the last call
    sub_time = time (NULL) - last_time;
    // Set time for next iteration
    last_time = time (NULL);
  }

// Go through active list one by one searching for timeouts
  for (i = 0; i < MAX_PROXY_CONS; i++) {
    if (proxy_con[i].active == 1) {
      // If the time has expired, then deactive the entry 
      if (proxy_con[i].timeout <= sub_time) {
        //printf ("Proxy: %s timed out\n", proxy_con[i].src_jid);
        proxy_con[i].timeout = 0;
        proxy_con[i].active = 0;
        if (proxy_con[i].connection != NULL)
          close_xmpp_client (proxy_con[i].connection);
      }
      //  Subtract time off of timeout
      else
        proxy_con[i].timeout -= sub_time;
    }
  }

}

//XMLParser func called whenever the start of an XML element is encountered
static void XMLCALL startElement (void *data, const char *element_name,
                                  const char **attr)
{
  int i;

  in_body = 0;

  if (strcmp (element_name, "body") == 0) {
    in_body = 1;
    tmp_msg[0] = '\0';
  }


  if (strcmp (element_name, "message") == 0) {
    for (i = 0; attr[i]; i += 2) {
      const char *attr_name = attr[i];
      if (strcmp (attr_name, "from") == 0)
        strcpy (tmp_src_jid, attr[i + 1]);
      if (strcmp (attr_name, "to") == 0)
        strcpy (tmp_dst_jid, attr[i + 1]);
    }

  }

// Build message destine for node here
  //printf ("element: %s\n", element_name);
  for (i = 0; attr[i]; i += 2) {
    const char *attr_name = attr[i];
    //printf ("attr_name: %s\n", attr_name);
    //printf ("value: %s\n", attr[i + 1]);



  }

}

//XMLParser func called whenever an end of element is encountered
static void XMLCALL endElement (void *data, const char *element_name)
{
  int i;
  uint8_t v;
  char hex_block[3];
  uint32_t mac_addr;

  if (in_body == 1) {
    SAMPL_PEER_2_PEER_PKT_T p2p_pkt;
    //printf( "charData %d: ",len );
    //if(len>=100) { in_body=0; return; }
    /*for(i=0; i<len; i++ ) 
       {
       tmp_msg[i]=s[i];
       printf( "%c",s[i] );
       } */
    //tmp_msg[i]='\0';
    //printf( "\n" );
    tmp_pass[0] = '\0';


    p2p_pkt.pkt_type = XMPP_PKT;
    p2p_pkt.ctrl_flags = MOBILE_MASK | DEBUG_FLAG;
    p2p_pkt.ack_retry = 0x00;
    p2p_pkt.ttl = 5;

    // FIXME: Need to support arbitrary JIDs

    // Get network address by parsing first 8 characters of JID
/*        hex_block[2]='\0'; hex_block[0]=tmp_dst_jid[0];	hex_block[1]=tmp_dst_jid[1];
	sscanf( hex_block,"%02x", &v );
	p2p_pkt.dst_subnet_mac[2]=v;
	hex_block[0]=tmp_dst_jid[2];	hex_block[1]=tmp_dst_jid[3];
	sscanf( hex_block,"%02x", &v );
	p2p_pkt.dst_subnet_mac[1]=v;
	hex_block[0]=tmp_dst_jid[4];	hex_block[1]=tmp_dst_jid[5];
	sscanf( hex_block,"%02x", &v );
	p2p_pkt.dst_subnet_mac[0]=v;
	hex_block[0]=tmp_dst_jid[6];	hex_block[1]=tmp_dst_jid[7];
	sscanf( hex_block,"%02x", &v);
	p2p_pkt.dst_mac=v;
*/

    // Get reply JID from proxy connection cache
    // Trim off resource
    //printf ("Finding mac associated with %s\n", tmp_dst_jid);
    for (i = 0; i < strlen (tmp_dst_jid); i++)
      if (tmp_dst_jid[i] == '/') {
        tmp_dst_jid[i] = '\0';
        break;
      }
    //printf ("calling proxy find now\n");
    mac_addr = proxy_find_mac_addr (tmp_dst_jid);

    if (mac_addr == 0) {
      //printf ("*** <%s> not found in reply jid cache\n", tmp_dst_jid);
      return;
    }

    p2p_pkt.dst_mac = mac_addr & 0xff;
    p2p_pkt.dst_subnet_mac[0] = (mac_addr >> 8) & 0xff;
    p2p_pkt.dst_subnet_mac[1] = (mac_addr >> 16) & 0xff;
    p2p_pkt.dst_subnet_mac[2] = (mac_addr >> 24) & 0xff;



    p2p_pkt.src_subnet_mac[0] = gw_subnet_0;
    p2p_pkt.src_subnet_mac[1] = gw_subnet_1;
    p2p_pkt.src_subnet_mac[2] = gw_subnet_2;
    p2p_pkt.src_mac = 0;
    p2p_pkt.last_hop_mac = 0;
    p2p_pkt.buf = proxy_msg_buf;
    p2p_pkt.buf_len = P2P_PAYLOAD_START;
    p2p_pkt.seq_num = seq_num;
    seq_num++;
    p2p_pkt.priority = 0;
    p2p_pkt.check_rate = 100;
    p2p_pkt.payload = &(proxy_msg_buf[P2P_PAYLOAD_START]);

    // build xmpp message here
    xmpp_pkt_in.binary_flag = 0;
    xmpp_pkt_in.pub_sub_flag = 0;
    xmpp_pkt_in.explicit_src_jid_flag = 1;
    xmpp_pkt_in.timeout = 0;
    xmpp_pkt_in.passwd = tmp_pass;
    xmpp_pkt_in.src_jid = tmp_src_jid;
    xmpp_pkt_in.msg = tmp_msg;
    xmpp_pkt_in.src_jid_size = strlen (tmp_src_jid) + 1;
    xmpp_pkt_in.msg_size = strlen (tmp_msg) + 1;
    xmpp_pkt_in.passwd_size = strlen (tmp_pass) + 1;

    //printf ("src jid=%s msg=%s\n", tmp_src_jid, tmp_msg);
    p2p_pkt.payload_len = xmpp_pkt_pack (&xmpp_pkt_in, p2p_pkt.payload, 0);

    //printf ("msg len = %d\n", strlen (tmp_msg));
    if (p2p_pkt.payload_len != 0 && strlen (tmp_msg) != 0) {
      // Pack data structure values in buffer before transmit
      pack_peer_2_peer_packet (&p2p_pkt);
      tx_q_add (p2p_pkt.buf, p2p_pkt.buf_len,0,NULL);
    }
  }
  in_body = 0;


}

static void charData (void *userData, const char *s, int len)
{
  if (in_body == 1)
    strncat (tmp_msg, s, len);
}


void proxy_msg_handler (LmMessage * message)
{
  LmMessageNode *child = NULL;
  XML_Parser p;
  char *buf;
/*  child = lm_message_node_find_child(message->node,"event");
  if(child == NULL)
  {
    printf( "child is null\n" );
    return;
  }
*/
  child = lm_message_get_node (message);

  buf = lm_message_node_to_string (child);

  //printf( "buf=%s\n",buf ); 

  //Creates an instance of the XML Parser to parse the event packet
  p = XML_ParserCreate (NULL);
  if (!p) {
    log_write ("Couldnt allocate memory for XML Parser\n");
    return;
  }
  //Sets the handlers to call when parsing the start and end of an XML element
  XML_SetElementHandler (p, startElement, endElement);
  XML_SetCharacterDataHandler (p, charData);

  if (XML_Parse (p, buf, strlen (buf), 1) == XML_STATUS_ERROR) {
    sprintf (global_error_msg, "XMPP proxy XML parse error at line %u: %s",
             XML_GetCurrentLineNumber (p),
             XML_ErrorString (XML_GetErrorCode (p)));
    log_write (global_error_msg);
    XML_ParserFree (p);
    return;
  }

  XML_ParserFree (p);

}



void proxy_login_and_send (uint32_t mac_addr, char *src_jid, char *passwd,
                           char *dst_jid, char *msg, uint8_t len,
                           uint8_t txt_mode, uint16_t timeout)
{
  int i, con, active, ret;

  active = 0;
  con = -1;
  for (i = 0; i < MAX_PROXY_CONS; i++) {
    if (proxy_con[i].active == 1) {
      // found jid in active list
      if (strcmp (proxy_con[i].src_jid, src_jid) == 0) {
        con = i;
        active = 1;
        //lm_connection_close(proxy_con[i].connection,NULL );
        break;
      }
    }
    else
      con = i;
  }

  if (con == -1) {
    printf ("Sorry, no more proxy slots available!\r\n");
    return;
  }

// If the jid was not active, then open new connection 
  if (active == 0) {
    //sprintf( full_jid,"%s@%s",jid,p_server );
    //printf ("jid=%s passwd=%s p_server=%s p_port=%d p_ssl=%s p_pubsub=%s\n",
    //        src_jid, passwd, p_server, p_port, p_ssl_fingerprint, p_pubsub);
    proxy_con[con].connection =
      start_xmpp_client (src_jid, passwd, p_server, p_port, p_ssl_fingerprint,
                         p_pubsub, proxy_msg_handler);
    if (proxy_con[con].connection == NULL) {
      log_write ("Could not start xmpp proxy client:");
    sprintf (global_error_msg, "-> jid=%s passwd_len=%d server=%s port=%d ssl=%s pubsub=%s\n",
            src_jid, strlen(passwd), p_server, p_port, p_ssl_fingerprint, p_pubsub);
      log_write (global_error_msg);
      return -1;
    }
    strcpy (proxy_con[con].src_jid, src_jid);
    //strcpy(proxy_con[con].dst_jid,dst_jid);
    proxy_con[con].mac_addr = mac_addr;
    proxy_con[con].active = 1;
  }

// refresh timeout
  proxy_con[con].timeout = timeout;
//  printf ("setting timeout to %d\n", timeout);

  if (txt_mode == 1) {
    // Convert to hex
    for (i = 0; i < len; i++)
      sprintf (&(msg_buf[i * 2]), "%02x", (uint8_t) msg[i]);
    ret = send_direct_message (proxy_con[con].connection, dst_jid, msg_buf);
  }
  else
    ret = send_direct_message (proxy_con[con].connection, dst_jid, msg);

  if (ret != XMPP_NO_ERROR) {
    proxy_con[con].timeout = 0;
    proxy_con[con].active = 0;
    close_xmpp_client (proxy_con[i].connection);
    sprintf (global_error_msg, "XMPP Error: %s", ERROR_MESSAGE (ret));
    log_write (global_error_msg);
    sprintf (global_error_msg, "-> jid=%s passwd_len=%d server=%s port=%d ssl=%s pubsub=%s\n",
            src_jid, strlen(passwd), p_server, p_port, p_ssl_fingerprint, p_pubsub);
    log_write (global_error_msg);
  }

}

void proxy_login_and_publish (uint32_t mac_addr, char *src_jid, char *passwd,
                              char *event_node, char *msg, uint8_t len,
                              uint8_t txt_mode, uint16_t timeout)
{
  int ret, i;
  char timeStr[100];
  time_t timestamp;
  XMPPConnection *connection;
  char buf[1024];
  int con, active;

  active = 0;
  con = -1;
  for (i = 0; i < MAX_PROXY_CONS; i++) {
    if (proxy_con[i].active == 1) {
      // found jid in active list
      if (strcmp (proxy_con[i].src_jid, src_jid) == 0) {
        con = i;
        active = 1;
        //lm_connection_close(proxy_con[i].connection,NULL );
        break;
      }
    }
    else
      con = i;
  }

  if (con == -1) {
    log_write ("Out of proxy slots");
    return;
  }

// If the jid was not active, then open new connection 
  if (active == 0) {
    //sprintf( full_jid,"%s@%s",jid,p_server );
    //printf ("jid=%s passwd=%s p_server=%s p_port=%d p_ssl=%s p_pubsub=%s\n",
    //        src_jid, passwd, p_server, p_port, p_ssl_fingerprint, p_pubsub);
    proxy_con[con].connection =
      start_xmpp_client (src_jid, passwd, p_server, p_port, p_ssl_fingerprint,
                         p_pubsub, proxy_msg_handler);
    if (proxy_con[con].connection == NULL) {
      log_write ("Could not start xmpp proxy client:");
    sprintf (global_error_msg, "-> jid=%s passwd_len=%d server=%s port=%d ssl=%s pubsub=%s\n",
            src_jid, strlen(passwd), p_server, p_port, p_ssl_fingerprint, p_pubsub);
      log_write (global_error_msg);
      return -1;
    }
    strcpy (proxy_con[con].src_jid, src_jid);
    //strcpy(proxy_con[con].dst_jid,dst_jid);
    proxy_con[con].mac_addr = mac_addr;
    proxy_con[con].active = 1;
  }

//if(strstr(event_node,"@")==NULL )
//{
//sprintf( event_node,"event_node@%s",p_server );
//}
// refresh timeout
  proxy_con[con].timeout = timeout;



//      printf( "jid=%s passwd=%s p_server=%s p_port=%d p_ssl=%s p_pubsub=%s\n",jid, 
//                                              passwd,p_server,
//                                              p_port, p_ssl_fingerprint, 
//                                              p_pubsub );

  connection = proxy_con[con].connection;
  if (connection == NULL) {
    proxy_con[con].timeout = 0;
    proxy_con[con].active = 0;
    log_write ("Connection is NULL in xmpp proxy 1\n");
    return -1;
  }
  time (&timestamp);
  strftime (timeStr, 100, "%Y-%m-%d %X", localtime (&timestamp));
  if (txt_mode == 1) {
    for (i = 0; i < len; i++)
      sprintf (&(msg_buf[i * 2]), "%02x", (uint8_t) msg[i]);

    sprintf (buf,
             "<Node id=\"%s\" type=\"FIREFLY MOBILE\" timestamp=\"%s\"><hex_message msg=\"%s\"/></Node>",
             event_node, timeStr, msg_buf);
  }
  else
    sprintf (buf,
             "<Node id=\"%s\" type=\"FIREFLY MOBILE\" timestamp=\"%s\"><ascii_message msg=\"%s\"/></Node>",
             event_node, timeStr, msg);
  ret = publish_to_node (connection, event_node, buf);
  if (ret != XMPP_NO_ERROR) {
    proxy_con[con].timeout = 0;
    proxy_con[con].active = 0;
    sprintf (global_error_msg, "XMPP Error: %s", ERROR_MESSAGE (ret));
    log_write (global_error_msg);
    sprintf (global_error_msg, "-> jid=%s passwd_len=%d server=%s port=%d ssl=%s pubsub=%s\n",
            src_jid, strlen(passwd), p_server, p_port, p_ssl_fingerprint, p_pubsub);
    log_write (global_error_msg);
    close_xmpp_client (proxy_con[con].connection);
  }

//    lm_connection_close(connection,NULL);

}
