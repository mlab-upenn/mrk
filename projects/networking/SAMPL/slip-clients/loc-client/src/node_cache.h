#ifndef _NODE_LIST_H_
#define _NODE_LIST_H_


#define MAX_NODE_LEN	  32
#define MAX_NODE_ELEMENTS 256 


int reg_id_get(char *node_name,char *reg_id);
int reg_id_load_from_file(char *node_name);

int node_list_add(char *name);
int node_list_exists(char *name);
void node_list_init();
void check_and_create_node(char *node_name);

#endif
