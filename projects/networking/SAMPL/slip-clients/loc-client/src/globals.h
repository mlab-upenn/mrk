#ifndef __GLOBALS_H_
#define __GLOBALS_H_

#include <stdint.h>
#include <loc_engine.h>

#if SOX_SUPPORT
  #include <soxlib.h>
#endif

#define MAX_PAYLOAD	110

#define ERROR_LEVEL 1
#define WARNING_LEVEL 2

uint8_t gw_subnet_2;
uint8_t gw_subnet_1;
uint8_t gw_subnet_0;
uint8_t gw_mac;

nlist_t g_nlist;

char loc_db_path[64];
char beacon_path[64];
char map_path[64];


char registry_file_name[128];
char subscribe_file_name[128];
uint8_t debug_txt_flag;
uint8_t xmpp_flag;
uint8_t db_flag;
uint8_t log_level;
uint8_t slip_debug_flag;
uint8_t print_input_flag;

#if SOX_SUPPORT
XMPPConnection *connection;
#endif
#endif
