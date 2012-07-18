#include <pkt.h>
#include <stdint.h>
#include <stdio.h>


int pkt_to_buf (PKT_T * pkt, uint8_t * buf)
{
  uint8_t i, checksum;
  if (buf == NULL || pkt == NULL)
    return -1;
  buf[PKT_TYPE_INDEX] = pkt->type;
  buf[PKT_SRC_MAC_0_INDEX] = pkt->src_mac & 0xff;
  buf[PKT_SRC_MAC_1_INDEX] = (pkt->src_mac >> 8) & 0xff;
  buf[PKT_SRC_MAC_2_INDEX] = (pkt->src_mac >> 16) & 0xff;
  buf[PKT_SRC_MAC_3_INDEX] = (pkt->src_mac >> 24) & 0xff;
  buf[PKT_DST_MAC_0_INDEX] = pkt->dst_mac & 0xff;
  buf[PKT_DST_MAC_1_INDEX] = (pkt->dst_mac >> 8) & 0xff;
  buf[PKT_DST_MAC_2_INDEX] = (pkt->dst_mac >> 16) & 0xff;
  buf[PKT_DST_MAC_3_INDEX] = (pkt->dst_mac >> 24) & 0xff;
  buf[PKT_PAYLOAD_LEN_INDEX] = pkt->payload_len;
  checksum = 0;
  for (i = 0; i < PKT_PAYLOAD_START_INDEX; i++)
    checksum += buf[i];
  for (i = 0; i < pkt->payload_len; i++) {
    checksum += pkt->payload[i];
    buf[PKT_PAYLOAD_START_INDEX + i] = pkt->payload[i];
  }
  buf[PKT_PAYLOAD_START_INDEX + pkt->payload_len] = checksum;
  pkt->checksum = checksum;
  // +1 at the end of the checksum
  return (PKT_PAYLOAD_START_INDEX + pkt->payload_len + 1);

}


int buf_to_pkt (uint8_t * buf, PKT_T * pkt)
{
  uint8_t i, checksum;

  if (buf == NULL || pkt == NULL)
    return -1;
  pkt->type = buf[PKT_TYPE_INDEX];
  pkt->src_mac = ((uint32_t) buf[PKT_SRC_MAC_3_INDEX] << 24) |
    ((uint32_t) buf[PKT_SRC_MAC_2_INDEX] << 16) |
    ((uint32_t) buf[PKT_SRC_MAC_1_INDEX] << 8) | buf[PKT_SRC_MAC_0_INDEX];

  pkt->dst_mac = ((uint32_t) buf[PKT_DST_MAC_3_INDEX] << 24) |
    ((uint32_t) buf[PKT_DST_MAC_2_INDEX] << 16) |
    ((uint32_t) buf[PKT_DST_MAC_1_INDEX] << 8) | buf[PKT_DST_MAC_0_INDEX];

  pkt->payload_len = buf[PKT_PAYLOAD_LEN_INDEX];
  checksum = 0;
  for (i = 0; i < pkt->payload_len; i++) {
    pkt->payload[i] = buf[PKT_PAYLOAD_START_INDEX + i];
    checksum += pkt->payload[i];
  }

  for (i = 0; i < PKT_PAYLOAD_START_INDEX; i++)
    checksum += buf[i];

  if (checksum == (buf[PKT_PAYLOAD_START_INDEX + pkt->payload_len]))
    return 1;

  return -1;

}
