/* FFDB - Maxim Buevich - July,2009
 * SAGA library for maintaining Firefly Sqlite3 databases.
 */


#ifndef _ffdb_
#define _ffdb_


#include <stdlib.h>
#include <stdio.h>
#include <sqlite3.h>
#include <string.h>
#include <malloc.h>
#include <unistd.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <limits.h>

#define SMALL_RAND (random()/(RAND_MAX/255))

#define BUFLEN 256
#define ERROR_RETRIES 10 


typedef struct firefly_env{
	char *id;
	unsigned int time;
	int light;
	int temp;
	int accl;
	int voltage;
	int audio;
	int arb1;
	int arb2;
} FF_ENV_PKT;

typedef struct power_meter{
	char *id;
	unsigned int time;
	int state;
	int rms_current;
	int rms_voltage;
	int true_power;
	int energy;
} FF_POW_PKT;

struct generic_integer_sensor{
	char *id;
	unsigned int time;
	char *type;
	int value;
};

struct firefly_stats{
	char *id;
	unsigned int time;
	int tx_pkts;
	int rx_pkts;
	int uptime;
	int deep_sleep;
	int idle_time;
	int sensor_samples;
};

struct location{
	char *id;
	unsigned int time;
	char *loc;
};

struct neighbor_list{
	char *id;
	unsigned int time;
	char *neighbor;
};



/* primary db functions */
void init_db(char *db_path);
void write_ff_env(struct firefly_env ff);
void write_power(struct power_meter pm );
void write_generic_integer(struct generic_integer_sensor gen);
void write_ff_stats(struct firefly_stats stats);
void write_location(struct location lc);
void write_neighbor_list(struct neighbor_list nl);

int get_last_write_time();
void set_device_alias(char *id, char *alias);

/* group editing functions */
void create_group(char *group_name, char *group_type);
void remove_group(char *group_name);
void add_to_group(char *id, char *group_name);
void remove_from_group(char *id, char *group_name);
void rename_group(char *old_group_name, char *new_group_name);

/* db helper functions */
void setup_new_db();
int table_exists(sqlite3 *db_name, char *table_name);
int device_exists(char *table_name);
static int callback(void *NotUsed, int argc, char **argv, char **azColName);
void print_table(char **result, int rownum, int colnum);
void build_ffs(struct firefly_env *ff_env, struct power_meter *pm);
void increment_ffs(struct firefly_env *ff_env, struct power_meter *pm);

/* globals */
sqlite3 *db;
sqlite3 *db_info;
sqlite3 *db_backup;


#endif
