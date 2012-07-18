#ifndef _PKT_H_
#define _PKT_H_

#include <stdint.h>

#define PKT_TYPE_INDEX		0
#define PKT_SRC_MAC_0_INDEX	1
#define PKT_SRC_MAC_1_INDEX	2
#define PKT_SRC_MAC_2_INDEX	3
#define PKT_SRC_MAC_3_INDEX	4
#define PKT_DST_MAC_0_INDEX	5
#define PKT_DST_MAC_1_INDEX	6
#define PKT_DST_MAC_2_INDEX	7
#define PKT_DST_MAC_3_INDEX	8
#define PKT_PAYLOAD_LEN_INDEX	9
#define PKT_PAYLOAD_START_INDEX 10

#define UNUSED	0
#define PING	1
#define APP	2

#define MAX_PAYLOAD	100



typedef struct {
  uint8_t type;
  uint32_t src_mac;
  uint32_t dst_mac;
  uint8_t payload_len;
  uint8_t payload[MAX_PAYLOAD];
  uint8_t checksum;
} PKT_T;

int pkt_to_buf (PKT_T * pkt, uint8_t * buf);
int buf_to_pkt (uint8_t * buf, PKT_T * pkt);




#endif
