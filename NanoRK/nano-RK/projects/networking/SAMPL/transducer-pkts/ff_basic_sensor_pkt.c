#include "ff_basic_sensor_pkt.h"
#include "transducer_registry.h"
#include <transducer_pkt.h>

uint8_t ff_basic_sensor_short_pack (TRANSDUCER_MSG_T * pkt,
                                    FF_SENSOR_SHORT_PKT_T * s)
{
  pkt->type = TRAN_FF_BASIC_SHORT;
  pkt->len = sizeof (FF_SENSOR_SHORT_PKT_T);
  pkt->payload[0] = s->battery;
  pkt->payload[1] = s->light;
  pkt->payload[2] = s->temperature;
  pkt->payload[3] = s->acceleration;
  pkt->payload[4] = s->sound_level;
  return 1;
}

uint8_t ff_basic_sensor_short_unpack (TRANSDUCER_MSG_T * t,
                                      FF_SENSOR_SHORT_PKT_T * s)
{
  s->battery = (uint8_t)t->payload[0];
  s->light =(uint8_t) t->payload[1];
  s->temperature = (uint8_t)t->payload[2];
  s->acceleration = (uint8_t)t->payload[3];
  s->sound_level = (uint8_t)t->payload[4];
  return 1;
}
