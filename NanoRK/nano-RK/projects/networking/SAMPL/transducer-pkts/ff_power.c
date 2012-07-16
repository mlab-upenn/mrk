#include "ff_power.h"
#include "transducer_registry.h"
#include <transducer_pkt.h>
#include <stdio.h>

uint8_t ff_power_config_pack(uint8_t *payload,
                                    FF_POWER_CONFIG_PKT * p)
{
	payload[0]=CONFIG_PKT;
	payload[1]=p->socket;
	payload[2]=p->push_enable;
	payload[3]=p->push_threshold;
  return 4;
}


uint8_t ff_power_virtual_pack(uint8_t *payload,
                                    FF_POWER_VIRTUAL_PKT * p)
{
	payload[0]=VIRTUAL_PKT;
	payload[1]=p->state;
	payload[2]=p->last_state;
	payload[3]=p->event;
	payload[4]=p->time[0];
	payload[5]=p->time[1];
	payload[6]=p->time[2];
	payload[7]=p->time[3];
	payload[8]=p->time[4];
	payload[9]=p->time[5];
	payload[10]=p->time[6];
	payload[11]=p->time[7];

  return 12;
}

uint8_t ff_power_virtual_unpack(uint8_t *payload,
                                    FF_POWER_VIRTUAL_PKT * p)
{
	if(payload[0]!=VIRTUAL_PKT) return 0;
	if(p==NULL || payload==NULL) return 0;
	p->type=payload[0];
	p->state=payload[1];
	p->last_state=payload[2];
	p->event=payload[3];
	p->time[0]=payload[4];
	p->time[1]=payload[5];
	p->time[2]=payload[6];
	p->time[3]=payload[7];
	p->time[4]=payload[8];
	p->time[5]=payload[9];
	p->time[6]=payload[10];
	p->time[7]=payload[11];
  return 12;
}


uint8_t ff_power_config_unpack(uint8_t *payload,
                                    FF_POWER_CONFIG_PKT * p)
{
	if(payload[0]!=CONFIG_PKT) return 0;
	if(p==NULL || payload==NULL) return 0;
	p->type=payload[0];
	p->socket=payload[1];
	p->push_enable=payload[2];
	p->push_threshold=payload[3];
  return 4;
}

uint8_t ff_power_actuate_pack(uint8_t *payload,
                                    FF_POWER_ACTUATE_PKT * p)
{
	payload[0]=ACTUATE_PKT;
	payload[1]=p->socket0_state;
	payload[2]=p->socket1_state;
  return 3;
}

uint8_t ff_power_actuate_unpack(uint8_t *payload,
                                    FF_POWER_ACTUATE_PKT * p)
{
	if(payload[0]!=ACTUATE_PKT) return 0;
	if(p==NULL || payload==NULL) return 0;
	p->type=payload[0];
	p->socket0_state=payload[1];
	p->socket1_state=payload[2];
  return 3;
}


uint8_t ff_power_rqst_pack(uint8_t *payload,
                                    FF_POWER_RQST_PKT * p)
{
	payload[0]=SENSE_RQST_PKT;
	payload[1]=p->socket;
	payload[2]=p->pkt_type;
  return 3;
}

uint8_t ff_power_rqst_unpack(uint8_t *payload,
                                    FF_POWER_RQST_PKT * p)
{
	if(payload[0]!=SENSE_RQST_PKT) return 0;
	if(p==NULL || payload==NULL) return 0;
	p->type=payload[0];
	p->socket=payload[1];
	p->pkt_type=payload[2];
  return 3;
}





uint8_t ff_power_sense_pack(uint8_t *payload,
                                    FF_POWER_SENSE_PKT * p)
{
	payload[0]=SENSE_PKT;
	payload[1]=(p->socket_num<<4) | (p->socket1_state<<1) | p->socket0_state;
	payload[2]=(p->rms_current>>8)&0xff;
        payload[3]=p->rms_current&0xff;
	payload[4]=(p->rms_voltage>>8)&0xff;
        payload[5]=p->rms_voltage&0xff;
        payload[6]=(p->true_power>>16)&0xff;
        payload[7]=(p->true_power>>8)&0xff;
        payload[8]=(p->true_power)&0xff;
        payload[9]=p->energy[0];
        payload[10]=p->energy[1];
        payload[11]=p->energy[2];
        payload[12]=p->energy[3];
        payload[13]=p->energy[4];
        payload[14]=p->energy[5];
  return 15;
}

uint8_t ff_power_sense_unpack(uint8_t *payload,
                                    FF_POWER_SENSE_PKT * p)
{
	if(payload[0]!=SENSE_PKT) return 0;
	if(p==NULL || payload==NULL) return 0;
	p->type=payload[0];
	p->socket_num=payload[1]>>4;
	p->socket0_state=payload[1]&0x1;
	p->socket1_state=(payload[1]&0x2)>>1;
	p->rms_current=((uint16_t)payload[2]<<8) | (uint16_t)payload[3];
	p->rms_voltage=((uint16_t)payload[4]<<8) | (uint16_t)payload[5];
	p->true_power=((uint32_t)payload[6]<<16) | ((uint32_t)payload[7]<<8) | (uint32_t)payload[8];
	//p->energy=((uint32_t)payload[9]<<24) | ((uint32_t)payload[10]<<16) | ((uint32_t)payload[11]<<8) | (uint32_t)payload[12];
	p->energy[0]=payload[9];
	p->energy[1]=payload[10];
	p->energy[2]=payload[11];
	p->energy[3]=payload[12];
	p->energy[4]=payload[13];
	p->energy[5]=payload[14];
  return 15;
}

uint8_t ff_power_debug_pack(uint8_t *payload,
                                    FF_POWER_DEBUG_PKT * p)
{
	payload[0]=DEBUG_PKT;
	payload[1]=(p->rms_current>>8)&0xff;
        payload[2]=p->rms_current&0xff;
        payload[3]=(p->true_power>>16)&0xff;
        payload[4]=(p->true_power>>8)&0xff;
        payload[5]=(p->true_power)&0xff;
        payload[6]=p->energy[0];
        payload[7]=p->energy[1];
        payload[8]=p->energy[2];
        payload[9]=p->energy[3];
        payload[10]=p->energy[4];
        payload[11]=p->energy[5];
        payload[12]=(p->current_p2p_high>>8)&0xff;
        payload[13]=(p->current_p2p_high)&0xff;
        payload[14]=(p->current_p2p_low>>8)&0xff;
        payload[15]=(p->current_p2p_low)&0xff;
        payload[16]=(p->rms_current2>>8)&0xff;
        payload[17]=p->rms_current2&0xff;
        payload[18]=(p->true_power2>>16)&0xff;
        payload[19]=(p->true_power2>>8)&0xff;
        payload[20]=(p->true_power2)&0xff;
        payload[21]=p->energy2[0];
        payload[22]=p->energy2[1];
        payload[23]=p->energy2[2];
        payload[24]=p->energy2[3];
        payload[25]=p->energy2[4];
        payload[26]=p->energy2[5];
        payload[27]=(p->current_p2p_high2>>8)&0xff;
        payload[28]=(p->current_p2p_high2)&0xff;
        payload[29]=(p->current_p2p_low2>>8)&0xff;
        payload[30]=(p->current_p2p_low2)&0xff;
        payload[31]=(p->voltage_p2p_high>>8)&0xff;
        payload[32]=(p->voltage_p2p_high)&0xff;
        payload[33]=(p->voltage_p2p_low>>8)&0xff;
        payload[34]=(p->voltage_p2p_low)&0xff;
        payload[35]=(p->rms_voltage>>8)&0xff;
        payload[36]=p->rms_voltage&0xff;
        payload[37]=p->freq&0xff;
        payload[38]=(p->total_secs>>24)&0xff;
        payload[39]=(p->total_secs>>16)&0xff;
        payload[40]=(p->total_secs>>8)&0xff;
        payload[41]=(p->total_secs)&0xff;
			  payload[42]=(p->socket1_state<<1) | p->socket0_state;
  return 43;
}

uint8_t ff_power_debug_unpack(uint8_t *payload,
                                    FF_POWER_DEBUG_PKT * p)
{
	if(payload[0]!=DEBUG_PKT) return 0;
	if(p==NULL || payload==NULL ) return 0;
	p->type=payload[0];
	p->rms_current=(payload[1]<<8) | payload[2];
	p->true_power=((uint32_t)payload[3]<<16) | ((uint32_t)payload[4]<<8) | (uint32_t)payload[5];
	//p->energy=((uint32_t)payload[6]<<24) | ((uint32_t)payload[7]<<16) | ((uint32_t)payload[8]<<8) | (uint32_t)payload[9];
	p->energy[0]=payload[6];
	p->energy[1]=payload[7];
	p->energy[2]=payload[8];
	p->energy[3]=payload[9];
	p->energy[4]=payload[10];
	p->energy[5]=payload[11];
	p->current_p2p_high=(payload[12]<<8) | payload[13];
	p->current_p2p_low=(payload[14]<<8) | payload[15];
	p->rms_current2=(payload[16]<<8) | payload[17];
	p->true_power2=((uint32_t)payload[18]<<16) | ((uint32_t)payload[19]<<8) | (uint32_t)payload[20];
	//p->energy2=((uint32_t)payload[19]<<24) | ((uint32_t)payload[20]<<16) | ((uint32_t)payload[21]<<8) | (uint32_t)payload[22];
	p->energy2[0]=payload[21];
	p->energy2[1]=payload[22];
	p->energy2[2]=payload[23];
	p->energy2[3]=payload[24];
	p->energy2[4]=payload[25];
	p->energy2[5]=payload[26];
	p->current_p2p_high2=(payload[27]<<8) | payload[28];
	p->current_p2p_low2=(payload[29]<<8) | payload[30];
	p->voltage_p2p_high=(payload[31]<<8) | payload[32];
	p->voltage_p2p_low=(payload[33]<<8) | payload[34];
	p->rms_voltage=(payload[35]<<8) | payload[36];
	p->freq = payload[37];
	p->total_secs=((uint32_t)payload[38]<<24) | ((uint32_t)payload[39]<<16) | ((uint32_t)payload[40]<<8) | (uint32_t)payload[41];
	p->socket0_state=payload[42]&0x1;
	p->socket1_state=(payload[42]&0x2)>>1;
  return 43;
}
