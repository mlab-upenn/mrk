#include <composer_handler.h>
#include <loudmouth/loudmouth.h>
#include <expat.h>
#include <stdint.h>
#include <error_log.h>
#include <sampl.h>
#include <transducer_pkt.h>
#include <transducer_registry.h>
#include <globals.h>
#include <power_request.h>


#define MAC_MASK	0x1
#define KEY_MASK	0x2
#define VALUE_MASK	0x4

#define MAX_TRANS_MSGS		32

static int from_pubsub_node,trans_mac, trans_key, trans_value, trans_state;

//static SAMPL_DOWNSTREAM_PKT_T t_ds_pkt;

static uint8_t pkt_buf[MAX_PAYLOAD];
static uint8_t tran_msg_cnt;
static uint8_t system_pkt;

static uint8_t event_node_set;
static uint8_t transducer_installation_set;
static uint8_t transducer_commands_set;
static char tran_name[32];

//XMLParser func called whenever the start of an XML element is encountered
static void XMLCALL startElement (void *data, const char *element_name,
                                  const char **attr)
{
  int i, j;
  unsigned int sensor_value = 0;
  char len, hex_digits[3];
/*
  printf( "element_name: %s\n",element_name );
    for (i = 0; attr[i]; i += 2) {
      const char *attr_name = attr[i];
      const char *attr_value = attr[i + 1];
	printf( "  Attribute name: %s\n",attr_name );
	printf( "  Attribute value: %s\n",attr_value);
      }
*/
  if(strcmp("message",element_name)==0) {
	event_node_set=0;
	transducer_installation_set=0;
	transducer_commands_set=0;
	}

  if(strcmp("items",element_name)==0) {
    for (i = 0; attr[i]; i += 2) {
      const char *attr_name = attr[i];
      const char *attr_value = attr[i + 1];
	if(strcmp(attr_name,"node")==0 ) 
		{
			strcpy(xmpp_in_event_node,attr_value );
			event_node_set=1;
		}
      }
   }


  if(strcmp("TransducerInstallation",element_name)==0) {
    for (i = 0; attr[i]; i += 2) {
      const char *attr_name = attr[i];
      const char *attr_value = attr[i + 1];
	if(strcmp(attr_name,"name")==0 ) strcpy(tran_name,attr_value );
	transducer_installation_set=1;
      }
   }

  if(strcmp("TransducerCommand",element_name)==0) {
      xmpp_in_num_params=0;
      for (i = 0; attr[i]; i += 2) {
      const char *attr_name = attr[i];
      const char *attr_value = attr[i + 1];
      strcpy(xmpp_in_attr_name[xmpp_in_num_params], attr_name ); 
      strcpy(xmpp_in_attr_value[xmpp_in_num_params], attr_value); 
      xmpp_in_num_params++;
     }
     transducer_commands_set=1;
   }




}




//XMLParser func called whenever an end of element is encountered
static void XMLCALL endElement (void *data, const char *element_name)
{

//printf( "END: %s\n",element_name );
if(strcmp(element_name,"message")==0 )
	{
	// Make sure all of the needed parameters were set
	if( event_node_set==0 || transducer_installation_set==0 || transducer_commands_set==0) return;
	
	if(strcmp(tran_name,"JigaWatt")==0 ) handle_power_request();

	}
}

static void charData (void *userData, const char *s, int len)
{

}



void handle_inbound_xmpp_msgs (LmMessage * message)
{
  LmMessageNode *child = NULL;
  XML_Parser p;
  gchar *buf;

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
  //XML_SetCharacterDataHandler(p, charData);  

  if (XML_Parse (p, buf, strlen (buf), 1) == XML_STATUS_ERROR) {
    sprintf (global_error_msg, "XMPP subscribe handler parse error at line %u: %s",
             XML_GetCurrentLineNumber (p),
             XML_ErrorString (XML_GetErrorCode (p)));
    log_write (global_error_msg);
    XML_ParserFree (p);
    return;
  }

  g_free (buf);
  XML_ParserFree (p);


}
