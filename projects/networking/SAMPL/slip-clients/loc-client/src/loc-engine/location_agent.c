/******************************************************************************
 *  Sensor-Over-XMPP(SOX) tools
 *  Copyright (C) 2008, Real-Time and Multimedia Lab, Carnegie Mellon University
 *  All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, version 2.0 of the License.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <expat.h>
#include <soxlib.h>
#include <nlist.h>
#include <loc_engine.h>
#include <sensor_data.h>

#define TIMESTAMP_MAX_CHARS 50
#define NODE_TYPE_MAX_CHARS 20

gboolean verbose = FALSE;
char current_node_id[NAME_MAX_CHARS];
char current_node_regid[NAME_MAX_CHARS];
char current_timestamp[TIMESTAMP_MAX_CHARS];
char current_node_type[NODE_TYPE_MAX_CHARS];
char current_link_mac[NAME_MAX_CHARS];
char current_rssi[NAME_MAX_CHARS];
nlist_t g_nlist;

char loc_db_path[64];
char beacon_path[64];
char map_path[64];
char data_blob[255];

void print_usage(char *prog_name)
{
  g_print("Usage: %s <-u username> <-p password> <-f event_node_file> [-l location-db-path] [-b beacon-path] [-m map-path][-host server_name] [-port server_port] [-pubsub pubsub_server_name] [-ssl \"server_ssl_fingerprint\"] [-verbose] [-unsubscribe]\n",prog_name);
  g_print("Usage: %s -help\n",prog_name);
  g_print("\t-l location-path= file containing signature database data\n");
  g_print("\t-b beacon-path= file containing beacon data\n");
  g_print("\t-m map-path= location to write map xml file\n");
  g_print("\t-f event_node_file = file containing names of event nodes to subscribe to\n");
  g_print("\t-u username = JID (give the full JID, i.e. user@domain)\n");
  g_print("\t-p password = JID user password\n");
  g_print("\t-host server_name = XMPP Server name (full server name) default=JID domain\n");
  g_print("\t-port server_port = XMPP Server port (usually 5222 for non-ssl, 5223 for ssl. Default=5223)\n");
  g_print("\t-pubsub pubsub_server_name = PubSub service address. Default=pubsub.<server_name>\n");
  g_print("\t-ssl server_ssl_fingerprint = SSL Fingerprint for XMPP Server, if connection uses ssl\n");
  g_print("\t-help = print this usage and exit\n");
  g_print("\t-verbose = print extra information\n");
  g_print("\t-unsubscribe = remove all current subscriptions\n");
  g_print("\nExample Usage\n");
  g_print("-------------\n");
  g_print("Subscribe to event nodes listed in test.xml:\n");
  g_print("  %s -u user@domain.org -p passwd -f test.xml\n");
  g_print("\nContents of test.xml for 4 different event nodes:\n");
  g_print("00000a04\n");
  g_print("00000a05\n");
  g_print("00000a06\n");
  g_print("00000a07\n");
} 

//XMLParser func called whenever the start of an XML element is encountered
static void XMLCALL startElement(void *data, const char *element_name, const char **attr) {
  int i;
  const char* node_desc = NULL;
  int tmp;

  if(strcmp(element_name,"DeviceInstallation")==0) {
    for(i=0;attr[i];i+=2) {
      const char* attr_name = attr[i];
      if(strcmp(attr_name,"id")==0)
	strcpy(current_node_id,attr[i+1]);
      else if(strcmp(attr_name,"regid")==0)
	strcpy(current_node_regid,attr[i+1]);
      else if(strcmp(attr_name,"type")==0)
	strcpy(current_node_type,attr[i+1]);
      else if(strcmp(attr_name,"description")==0)
	node_desc = attr[i+1];
      else if(strcmp(attr_name,"timestamp")==0)
	strcpy(current_timestamp,attr[i+1]);
    }
    g_print("DeviceInstallation: id=%s, regid=%s, desc=%s, timestamp=%s\n",current_node_id,current_node_regid,node_desc,current_timestamp);
    sscanf(current_node_id, "%X", &tmp);
    g_nlist.num=0;
    g_nlist.mac=tmp;
  }
  else if(strcmp(element_name,"DeviceConnection")==0) {
    for(i=0;attr[i];i+=2) {
      const char* attr_name = attr[i];
      if(strcmp(attr_name,"regid")==0)
	strcpy(current_node_regid,attr[i+1]);
      else if(strcmp(attr_name,"link")==0)
	strcpy(current_link_mac,attr[i+1]);
      else if(strcmp(attr_name,"rssi")==0)
	strcpy(current_rssi,attr[i+1]);
    }

    sscanf(current_link_mac, "%X", &tmp);
    g_nlist.link_mac[g_nlist.num]=tmp;
    sscanf(current_rssi, "%d", &tmp);
    g_nlist.rssi[g_nlist.num]=tmp;
    g_nlist.num++;
  }
  else if(strcmp(element_name,"TransducerValue")==0) {
  data_blob[0]=NULL;
    for(i=0;attr[i];i+=2) {
      const char* attr_name = attr[i];
      if(strcmp(attr_name,"regid")==0)
	strcpy(current_node_regid,attr[i+1]);
      else if(strcmp(attr_name,"rawValue")==0)
	strcpy(data_blob,attr[i+1]);
      else if(strcmp(attr_name,"rssi")==0)
	strcpy(current_rssi,attr[i+1]);
    }
  printf( "Got Hex data from %s: %s\n",current_node_id,data_blob );
  sscanf(current_node_id, "%X", &tmp);
  sensor_data_add(tmp,data_blob);
  }


 }

//XMLParser func called whenever an end of element is encountered
static void XMLCALL endElement(void *data, const char *element_name) {

  if(strcmp(element_name,"DeviceInstallation")==0) {
	loc_engine_update(&g_nlist);
	}
}

//Message handler called for every message received
static void handle_event(LmMessage* message) {
  LmMessageNode *child = NULL;
  XML_Parser p;
  char *buf;

//  child = lm_message_node_find_child(message->node,"DeviceInstallation");
   child = lm_message_get_node(message);
   if(child == NULL)
    return;

  buf = lm_message_node_to_string(child);

  if(verbose)
    g_print("got event\n%s\n",buf);

  //Creates an instance of the XML Parser to parse the event packet
  p = XML_ParserCreate(NULL);
  if(!p) {
    g_printerr("Couldnt allocate memory for XML Parser\n");
    return;
  }

  //Sets the handlers to call when parsing the start and end of an XML element
  XML_SetElementHandler(p,startElement,endElement);
  
  if(XML_Parse(p,buf,strlen(buf),1) == XML_STATUS_ERROR) {
    g_printerr("Parse Error at line %u:\n%s\n",
		XML_GetCurrentLineNumber(p),
		XML_ErrorString(XML_GetErrorCode(p)));
    return;
  }

  XML_ParserFree(p);
}

#define MAX_LINE 64 

gchar* readline(gchar *in, FILE* file) {
  gchar* cptr;
  
  if ((cptr = fgets(in, MAX_LINE, file)) != NULL) {
    while(*cptr == ' ' || *cptr == '\t') {
      cptr++;
    }
    in[strlen(in)-1]='\0';
    return cptr;
  } else {
    return NULL;
  }
}

//Subscribe wrapper
void do_subscribe(gpointer element, gpointer data)
{
  int ret = 0;
  NodeDesc *node_desc = (NodeDesc*)element;
  XMPPConnection *connection = (XMPPConnection*)data;
  if(verbose)
    g_print("Subscribing to Node: name=%s\n",node_desc->name);
  
  if((ret = subscribe_to_node(connection,node_desc->name)) != XMPP_NO_ERROR)
    g_printerr("Could not subscribe to node <%s>: %s\n",node_desc->name,ERROR_MESSAGE(ret));
}

//Unsubscribe wrapper
void do_unsubscribe(gpointer element, gpointer data)
{
  int ret = 0;
  NodeDesc *node_desc = (NodeDesc*)element;
  XMPPConnection *connection = (XMPPConnection*)data;
  if(verbose)
    g_print("current_subscription Node: name=%s, desc=%s\n",node_desc->name,node_desc->desc);
  
  if((ret = unsubscribe_from_node(connection,node_desc->name)) != XMPP_NO_ERROR)
    g_printerr("Could not unsubscribe from node <%s>: %s\n",node_desc->name,ERROR_MESSAGE(ret));
}

int main(int argc, char** argv) {
  gchar buf[64];
  gchar domain[80];
  gchar *result = NULL;

  gchar* username = NULL;
  gchar* password = NULL;
  gchar  xmpp_server[80];
  gchar* xmpp_ssl_fingerprint = "blah";
  gchar pubsub_server[80];
  int xmpp_server_port = 5223;

  gchar* event_node_file_str = NULL;
  FILE* event_node_file = NULL;
  gboolean unsubscribe = TRUE;
  gchar* ret_char = NULL;

  int current_arg_num = 1;
  gchar* current_arg_name = NULL;
  gchar* current_arg_val = NULL;

  GSList *node_list = NULL;
  GMainLoop *main_loop = NULL;

  XMPPConnection *connection = NULL;
  if(argc < 7 || argc > 17 || argv[1] == "-help") {
    g_printerr("Missing arguments\n");
    print_usage(argv[0]);
    return -1;
  }

  strcpy(xmpp_server,"none");
  strcpy(pubsub_server,"none");
  strcpy(loc_db_path,"config/loc-db.txt");
  strcpy(beacon_path,"config/beacons.txt");
  strcpy(map_path,"map.xml");

  while(current_arg_num < argc) {
    current_arg_name = argv[current_arg_num++];

    if(strcmp(current_arg_name,"-help")==0) {
      print_usage(argv[0]);
      return -1;      
    }

    if(strcmp(current_arg_name,"-verbose")==0) {
      verbose = TRUE;
      continue;
    }

    if(strcmp(current_arg_name,"-unsubscribe")==0) {
      unsubscribe = TRUE;
      continue;
    }    

    if(current_arg_num == argc) {
      print_usage(argv[0]);
      return -1;
    }
      
    current_arg_val = argv[current_arg_num++];

    if(strcmp(current_arg_name,"-f")==0) {
      event_node_file_str = current_arg_val;
    }
    else if(strcmp(current_arg_name,"-u")==0) {
      username = current_arg_val;
      if(strcmp(xmpp_server,"none")==0) {
	strcpy(domain,username);
	result = strtok(domain,"@");
	result = strtok(NULL,"@");
	strcpy(xmpp_server,result);
	strcpy(pubsub_server,"pubsub.");
	strcat(pubsub_server,xmpp_server);
      }
    }
    else if(strcmp(current_arg_name,"-p")==0) {
      password = current_arg_val;
    }
    else if(strcmp(current_arg_name,"-host")==0) {
      strcpy(xmpp_server,current_arg_val);
      if(strcmp(pubsub_server,"none")==0) {
	strcpy(pubsub_server,"pubsub.");
	strcat(pubsub_server,xmpp_server);
      }
    }
    else if(strcmp(current_arg_name,"-port")==0) {
      xmpp_server_port = atoi(current_arg_val);
    }
    else if(strcmp(current_arg_name,"-l")==0) {
      strcpy(loc_db_path,current_arg_val);
    }
    else if(strcmp(current_arg_name,"-b")==0) {
      strcpy(beacon_path,current_arg_val);
    }
    else if(strcmp(current_arg_name,"-m")==0) {
      strcpy(map_path,current_arg_val);
    }
    else if(strcmp(current_arg_name,"-pubsub")==0) {
      strcpy(pubsub_server,current_arg_val);
    }
    else if(strcmp(current_arg_name,"-ssl")==0) {
      xmpp_ssl_fingerprint = current_arg_val;
    }
    else {
      g_printerr("Unknown argument: %s\n",current_arg_name);
      print_usage(argv[0]);
      return -1;
    }
  }

  if(username == NULL) {
    g_printerr("Username missing\n");
    print_usage(argv[0]);
    return -1;
  }
  else if(password == NULL) {
    g_printerr("Password missing\n");
    print_usage(argv[0]);
    return -1;
  }
  else if(event_node_file_str == NULL) {
    g_printerr("EventNode filename is missing\n");
    print_usage(argv[0]);
    return -1;
  }

  if(verbose) {
    g_print("XMPP Server: %s\n",xmpp_server);
    g_print("XMPP Server Port: %d\n",xmpp_server_port);
    g_print("XMPP PubSub Server: %s\n",pubsub_server);
    g_print("XMPP Server SSL Fingerprint: %s\n",xmpp_ssl_fingerprint);
    g_print("username: %s\n",username);
    g_print("EventNode file: %s\n",event_node_file_str);
    g_print("Verbose: YES\n");
    g_print("\n");
  }

  connection = start_xmpp_client(username,
				 password,
				 xmpp_server,
				 xmpp_server_port,
				 xmpp_ssl_fingerprint,
				 pubsub_server,
				 handle_event);
  
  if(connection == NULL) {
    g_printerr("Could not start client\n");
    return -1;
  }

  if(verbose)
    printf("initialized xmpp client\n");  

  //Unsubscribe from all nodes on service
  if(unsubscribe) {    
    node_list = get_all_current_subscriptions(connection);
    if(node_list == NULL)
      g_print("No subscriptions found\n");
    else {
      g_slist_foreach(node_list,do_unsubscribe,connection);
      g_slist_free(node_list);
      node_list = NULL;
    }
  }
  
  //Read event nodes from file if specified by user
  if(event_node_file_str != NULL) {
    event_node_file = fopen(event_node_file_str,"r");
    if(event_node_file == NULL) {
      g_printerr("ERROR: could not open file %s\n",event_node_file_str);
      return -1;
    }
    while((ret_char=readline(buf,event_node_file))!=NULL) {
      NodeDesc *node = g_new0 (NodeDesc, 1);
      strcpy(node->name,ret_char);
      node_list = g_slist_prepend (node_list, node);
    }
    fclose(event_node_file);
  }

  loc_engine_init(loc_db_path,beacon_path, map_path);
  //Subscribe to all nodes in list
  if(node_list == NULL)
    g_print("No Nodes to subscribe to\n");
  else {
    g_slist_foreach(node_list,do_subscribe,connection);
    g_slist_free(node_list);

    main_loop = g_main_loop_new (NULL, FALSE);
    g_main_loop_run (main_loop);
  }

  return 0;
}
