#ifndef TRANSDUCER_XMPP_H_
#define TRANSDUCER_XMPP_H_
#include <sampl.h>

typedef struct xmpp_power
{
        uint8_t socket;  
        uint8_t state;  
        uint16_t adc_rms_current;
        uint16_t adc_rms_voltage;
        uint32_t adc_true_power;
        uint32_t adc_energy;
        float rms_current;
        float rms_voltage;
        float apparent_power;
        float true_power;
        float energy;
        float power_factor;
        uint8_t  freq;
} XMPP_POWER_T;


void publish_xmpp_transducer_pkt(SAMPL_GATEWAY_PKT_T *gw_pkt );
int send_xmpp_power(char *event_node, XMPP_POWER_T pwr  );

#endif
