#ifndef SLIP_SERVER_H_
#define SLIP_SERVER_H_

#include <stdint.h>

#define LAST_CONNECTION 0
#define STATIC_CLIENT 1 

void slipstream_server_open (int port,char *client_ip, uint8_t debug_flag);
uint8_t slipstream_server_non_blocking_rx (uint8_t *buf);
uint8_t slipstream_server_tx (uint8_t *buf, uint8_t size);

#endif
