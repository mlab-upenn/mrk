#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <slipstream.h>
#include <pkt.h>
#include <time.h>


#define NONBLOCKING  0
#define BLOCKING     1
#define POWER_SCALER	111.0
#define VOLTAGE_SCALER 	3.11
#define CURRENT_SCALER	36.0

typedef struct ff_power_pkt {
  uint32_t total_secs;
  uint8_t freq;

  uint16_t rms_voltage;
  uint16_t rms_current;
  uint32_t true_power;
  uint8_t energy[6];
  uint64_t long_energy;
  uint8_t socket_state;

  uint16_t rms_voltage1;
  uint16_t rms_current1;
  uint32_t true_power1;
  uint8_t energy1[6];
  uint64_t long_energy1;
  uint8_t socket_state1;
  uint16_t v_p2p_low;
  uint16_t v_p2p_high;
  uint16_t c_p2p_low;
  uint16_t c_p2p_high;
  uint16_t c_p2p_low2;
  uint16_t c_p2p_high2;
  uint16_t v_center;
  uint16_t c_center;
  uint16_t c2_center;
} FF_POWER_PKT;

typedef struct ff_env_pkt{
  uint16_t bat;
  uint16_t light;
  uint16_t temp;
  uint16_t acc_x;
  uint16_t acc_y;
  uint16_t acc_z;
  uint16_t audio_p2p;
  uint32_t digital_temp;
  uint32_t humidity;
  uint32_t pressure;
  uint16_t motion;
  uint8_t gpio_state;
} FF_ENV_PKT;



uint8_t power_unpack (uint8_t * payload, FF_POWER_PKT * p);
uint8_t env_unpack(uint8_t * payload, FF_ENV_PKT * p);
int outlet_actuate (uint32_t mac, uint8_t socket, uint8_t state);
int send_ping ();

int verbose;

int main (int argc, char *argv[])
{
  char buffer[128];
  int v, cnt, i, size, socket, state;
  uint32_t target_mac;
  FF_POWER_PKT my_pwr_pkt;
  FF_ENV_PKT my_env_pkt;
  PKT_T my_pkt;
  time_t ts;

  if (argc < 5) {
    printf ("Usage: server port mac-addr state [-v]\n");
    printf ("  mac-addr:  mac address in hex\n");
    printf ("  state:     1 for on, 0 for off\n");
    printf ("  Example:   ./plug-ctrl localhost 5000 0x1 1\n");
    exit (1);
  }

  state = 0;
  verbose=0;
	if(argc==6)
	{
		if(strcmp(argv[5],"-v")==0) { printf( "Verbose on\n" ); verbose=1; }
	}

	//target_mac=atoi(argv[3]);
	sscanf( argv[3],"%x",&target_mac );
	state=atoi(argv[4]);

  v = slipstream_open (argv[1], atoi (argv[2]), BLOCKING);

  //v = send_ping ();

  socket=0;
  if(verbose) printf ("Sending an actuate command [mac=0x%x socket %d, state %d]\n",target_mac,socket,state);
  // Enable outlet 0 on node 0x01
  outlet_actuate (target_mac, socket, state);



}



int outlet_actuate (uint32_t mac, uint8_t socket, uint8_t state)
{
  int v, size;
  PKT_T my_pkt;
  char buffer[128];

  my_pkt.src_mac = 0x00000000;
  my_pkt.dst_mac = mac;
  my_pkt.type = APP;
  my_pkt.payload[0] = 1;        // Number of Elements
  my_pkt.payload[1] = 2;        // KEY (2->actuate, 1-> Power Packet) 
  my_pkt.payload[2] = socket;   // Outlet number 0/1
  my_pkt.payload[3] = state;    // Outlet state 0/1
  my_pkt.payload_len = 4;
  size = pkt_to_buf (&my_pkt, buffer);
  v = slipstream_send (buffer, size);
  return v;
}

int send_ping (void)
{
  int v, size;
  PKT_T my_pkt;
  char buffer[128];
  // Send a ping at startup
  my_pkt.src_mac = 0x00000000;
  my_pkt.dst_mac = 0xffffffff;
  my_pkt.type = PING;
  my_pkt.payload_len = 0;
  size = pkt_to_buf (&my_pkt, buffer);
  v = slipstream_send (buffer, size);
  if (v == 0)
    printf ("Error sending\n");
  return v;
}


uint8_t env_unpack (uint8_t * payload, FF_ENV_PKT * p)
{
	int i;
p->bat = payload[2]<<8 | payload[3];
p->light = payload[4]<<8 | payload[5];
p->temp = payload[6]<<8 | payload[7];
p->acc_x = payload[8]<<8 | payload[9];
p->acc_y = payload[10]<<8 | payload[11];
p->acc_z = payload[12]<<8 | payload[13];
p->audio_p2p = payload[14]<<8 | payload[15];
p->humidity = payload[16]<<24 | payload[17]<< 16 | payload[18]<<8 | payload[19];
p->digital_temp= payload[20]<<24 | payload[21]<< 16 | payload[22]<<8 | payload[23];
p->pressure= payload[24]<<24 | payload[25]<< 16 | payload[26]<<8 | payload[27];
p->motion = payload[28]<<8 | payload[29];
p->gpio_state = payload[30];
}

uint8_t power_unpack (uint8_t * payload, FF_POWER_PKT * p)
{

  p->total_secs =
    ((uint32_t) payload[2] << 24) | ((uint32_t) payload[3] << 16) |
    ((uint32_t) payload[4] << 8) | (uint32_t) payload[5];
  p->freq = payload[6];
  p->rms_voltage = (payload[7] << 8) | payload[8];


  p->rms_current = (payload[9] << 8) | payload[10];
  p->true_power =
    ((uint32_t) payload[11] << 16) | ((uint32_t) payload[12] << 8) |
    (uint32_t) payload[13];
  p->energy[0] = payload[14];
  p->energy[1] = payload[15];
  p->energy[2] = payload[16];
  p->energy[3] = payload[17];
  p->energy[4] = payload[18];
  p->energy[5] = payload[19];
  p->long_energy = (uint64_t)payload[19]<<40 | (uint64_t)payload[18]<<32 | (uint64_t)payload[17]<<24 | payload[16]<<16 | payload[15]<<8 | payload[14];
  p->socket_state = payload[20];

  p->rms_current1 = (payload[21] << 8) | payload[22];
  p->true_power1 =
    ((uint32_t) payload[23] << 16) | ((uint32_t) payload[24] << 8) |
    (uint32_t) payload[25];
  
  p->energy1[0] = payload[26];
  p->energy1[1] = payload[27];
  p->energy1[2] = payload[28];
  p->energy1[3] = payload[29];
  p->energy1[4] = payload[30];
  p->energy1[5] = payload[31];
  p->long_energy1 =(uint64_t)payload[31]<<40 | (uint64_t)payload[30]<<32 | (uint64_t)payload[29]<<24 | payload[28]<<16 | payload[27]<<8 | payload[26];
  p->socket_state1 = payload[32];
  p->v_p2p_low = (uint16_t)payload[33]<<8 | (uint16_t) payload[34];
  p->v_p2p_high = (uint16_t)payload[35]<<8 | (uint16_t) payload[36];
  p->c_p2p_low= (uint16_t)payload[37]<<8 | (uint16_t) payload[38];
  p->c_p2p_high= (uint16_t)payload[39]<<8 | (uint16_t) payload[40];
  p->c_p2p_low2= (uint16_t)payload[41]<<8 | (uint16_t) payload[42];
  p->c_p2p_high2= (uint16_t)payload[43]<<8 | (uint16_t) payload[44];
  p->v_center= (uint16_t)payload[45]<<8 | (uint16_t) payload[46];
  p->c_center= (uint16_t)payload[47]<<8 | (uint16_t) payload[48];
  p->c2_center= (uint16_t)payload[49]<<8 | (uint16_t) payload[50];

}
