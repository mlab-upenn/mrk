#ifndef _COMPOSER_H_
#define _COMPOSER_H_
#include <stdint.h>
#include <loudmouth/loudmouth.h>

#define MAX_PARAMS	4

void handle_inbound_xmpp_msgs (LmMessage * message);

uint8_t xmpp_in_num_params;
char xmpp_in_attr_name[MAX_PARAMS][32];
char xmpp_in_attr_value[MAX_PARAMS][32];
char xmpp_in_event_node[64];

#endif
