#include <node_cache.h>
#include <stdint.h>
#include <time.h>
#include <string.h>
#include <stdio.h>
#include <globals.h>

char nodeIDs[MAX_NODE_ELEMENTS][MAX_NODE_LEN];
char registry_id[MAX_NODE_ELEMENTS][MAX_NODE_LEN];

uint8_t node_id_cnt;

void node_list_init ()
{
  int i;
  node_id_cnt = 0;
  for (i = 0; i < MAX_NODE_ELEMENTS; i++) {
    nodeIDs[i][0] = '\0';
    registry_id[i][0] = '\0';
  }
}


int node_list_exists (char *name)
{
  int i;
  if(debug_txt_flag==1 ) 
			printf ("node_id_cnt=%d\n", node_id_cnt);
  for (i = 0; i < node_id_cnt; i++)
    if (strcmp (name, nodeIDs[i]) == 0)
      return 1;
  return 0;
}

int node_list_add (char *name)
{

  if(debug_txt_flag==1 ) 
  printf ("Trying to add %s to node cache\n", name);
  if (node_id_cnt < MAX_NODE_ELEMENTS) {
    strcpy (nodeIDs[node_id_cnt], name);
    node_id_cnt++;
    return 1;
  }
  else {
  if(debug_txt_flag==1 ) 
    printf ("can't add %s, cache full\n", name);
    exit (0);
  }

  return 0;
}

int reg_id_get (char *node_name, char *reg_id)
{
  int i;

  if(debug_txt_flag==1 ) 
  printf ("searching for in cache: %s\n", node_name);
  for (i = 0; i < node_id_cnt; i++) {
    if (strcmp (nodeIDs[i], node_name) == 0 && *registry_id[i] != NULL) {
  if(debug_txt_flag==1 ) printf ("found reg id for %s\n", nodeIDs[i]);
      strcpy (reg_id, registry_id[i]);
      return 1;
    }

  }
  if(debug_txt_flag==1 ) 
  	printf ("registry node not found\n");
  return 0;
}

int reg_id_load_from_file (char *node_name)
{
  FILE *fp;
  char name[32], reg[32];
  int val, i;

  fp = fopen (registry_file_name, "r");
  if (fp == NULL) {
    printf ("no registry file: \"%s\"\n", registry_file_name);
    return 0;
  }
  if(debug_txt_flag==1 ) 
  printf ("Registry Open search for %s\n", node_name);
  do {
    val = fscanf (fp, "%s %s\n", name, reg);
//  printf( "scanning reg file %s, %s val=%d\n",name,reg,val );
    if (strcmp (name, node_name) == 0) {
      for (i = 0; i < node_id_cnt; i++) {
        if (strcmp (node_name, nodeIDs[i]) == 0) {
          // found node in list from file
          strcpy (registry_id[i], reg);
  				if(debug_txt_flag==1 ) 
             printf ("Adding reg id <%s> to event node <%s>\n", registry_id[i],
                  nodeIDs[i]);
          fclose (fp);
          return 1;
        }
      }
    }
  } while (val > 0);
  fclose (fp);
  return 0;
}

void check_and_create_node (char *node_name)
{
  int ret;
  char buf[1024];
  char time_str[64];
  char reg_id[64];
  char reg_name[64];
  time_t timestamp;
  SOXMessage *msg = NULL;

  // If I have already created the node this run
  if (node_list_exists (node_name) == 0) {
    // node doesn't exist
    // Add it to my list to stop creating new nodes
    ret = node_list_add (node_name);
    if (ret == 0) {
  	if(debug_txt_flag==1 ) 
      printf ("Ran out of local node cache!\r\n");
    }
    // generate parent node for gateway
  	if(debug_txt_flag==1 ) 
    			printf ("loading node from file!\n");

    reg_id_load_from_file (node_name);
    sprintf (reg_name, "%s_LIGHT", node_name);
    node_list_add (reg_name);
    reg_id_load_from_file (reg_name);
    sprintf (reg_name, "%s_TEMP", node_name);
    node_list_add (reg_name);
    reg_id_load_from_file (reg_name);
    sprintf (reg_name, "%s_AUDIO", node_name);
    node_list_add (reg_name);
    reg_id_load_from_file (reg_name);
    sprintf (reg_name, "%s_ACCEL", node_name);
    node_list_add (reg_name);
    reg_id_load_from_file (reg_name);
    sprintf (reg_name, "%s_BAT", node_name);
    node_list_add (reg_name);
    reg_id_load_from_file (reg_name);

    sprintf (reg_name, "%s_VOLTAGE_0", node_name); node_list_add (reg_name); reg_id_load_from_file (reg_name);
    sprintf (reg_name, "%s_CURRENT_0", node_name); node_list_add (reg_name); reg_id_load_from_file (reg_name);
    sprintf (reg_name, "%s_POWER_0", node_name); node_list_add (reg_name); reg_id_load_from_file (reg_name);
    sprintf (reg_name, "%s_APOWER_0", node_name); node_list_add (reg_name); reg_id_load_from_file (reg_name);
    sprintf (reg_name, "%s_POWER_FACTOR_0", node_name); node_list_add (reg_name); reg_id_load_from_file (reg_name);
    sprintf (reg_name, "%s_ENERGY_0", node_name); node_list_add (reg_name); reg_id_load_from_file (reg_name);
    sprintf (reg_name, "%s_STATE_0", node_name); node_list_add (reg_name); reg_id_load_from_file (reg_name);

    sprintf (reg_name, "%s_VOLTAGE_1", node_name); node_list_add (reg_name); reg_id_load_from_file (reg_name);
    sprintf (reg_name, "%s_CURRENT_1", node_name); node_list_add (reg_name); reg_id_load_from_file (reg_name);
    sprintf (reg_name, "%s_POWER_1", node_name); node_list_add (reg_name); reg_id_load_from_file (reg_name);
    sprintf (reg_name, "%s_APOWER_1", node_name); node_list_add (reg_name); reg_id_load_from_file (reg_name);
    sprintf (reg_name, "%s_POWER_FACTOR_1", node_name); node_list_add (reg_name); reg_id_load_from_file (reg_name);
    sprintf (reg_name, "%s_ENERGY_1", node_name); node_list_add (reg_name); reg_id_load_from_file (reg_name);
    sprintf (reg_name, "%s_STATE_1", node_name); node_list_add (reg_name); reg_id_load_from_file (reg_name);
    ret = create_event_node (connection, node_name, NULL, FALSE);
    if (ret != XMPP_NO_ERROR) {
      if (ret == XMPP_ERROR_NODE_EXISTS) {
        if (debug_txt_flag)
          printf ("Node '%s' already exists\n", node_name);
      }
      else {
        g_printerr ("Could not create event node '%s'. Error='%s'\n",
                    node_name, ERROR_MESSAGE (ret));
        return;
      }
    }
    else {

      // First time add a description of node
      // publish XML data for node

      if (debug_txt_flag)
        printf ("Created event node '%s'\n", node_name);

      time (&timestamp);
      strftime (time_str, 100, "%Y-%m-%dT%X", localtime (&timestamp));

      ret = reg_id_get (node_name, reg_id);
      if (ret == 0)
        strcpy (reg_id, "unknown");
      msg = create_sox_message ();
      msg_add_device_installation (msg, node_name, reg_id, "FIREFLY",
                                   "A Firefly Node", time_str);

      if (xmpp_flag == 1)
        ret = publish_sox_message (connection, node_name, msg);

      delete_sox_message (msg);
      if (ret != XMPP_NO_ERROR) {
        g_printerr ("Could not sent desc packet for '%s'. Error='%s'\n",
                    node_name, ERROR_MESSAGE (ret));
        return -1;
      }
    }
  }
  else
		if (debug_txt_flag) { printf ("Event Node exists\n"); }
}
