#ifndef XML_PKT_PARSER_H_
#define XML_PKT_PARSER_H_

#include <sampl.h>
#include <globals.h>

#define MAX_XML_FILE	4096

#define DS_PKT		0
#define P2P_PKT		1
#define SLEEP		2
#define TRANS_PKT	3

typedef struct gw_script_pkt 
{
int type;
int nav;
int size;
uint8_t pkt[MAX_PAYLOAD];
} GW_SCRIPT_PKT_T;

int build_sampl_pkts_from_xml(GW_SCRIPT_PKT_T *pkts, int max_pkts, char *xml_buf, int size);	
int load_xml_file(char *filename, char *xml_buf);

#endif
