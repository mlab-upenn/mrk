/* FFDB - Maxim Buevich - July,2009
 * SAGA library for maintaining Firefly Sqlite3 databases.
 * This is version 2.0, surely a significant achievement in the grand scheme of things...
 */

#include "ffdb.h"

/**** PRIMARY DB FUNCTIONS ****/

#if SQLITE_SUPPORT

/* 
 * opens db if one is present, otherwise creates
 * and prepares a new db.
 */
void init_db(char *db_path){
	int i, rc;
	char *dberror = NULL;
	char db_info_path[BUFLEN];
	char db_backup_path[BUFLEN];
	struct stat stat1;
	struct stat stat2;

	sprintf(db_info_path,"%s-info",db_path);
	sprintf(db_backup_path,"%s-backup",db_path);

	if(stat(db_path,&stat1) || stat(db_backup_path,&stat2)){		
		if(sqlite3_open(db_path,&db)){
			printf("Error opening main db.\n");
			return;
		}
		if(sqlite3_open(db_backup_path,&db_backup)){
			printf("Error opening backup db.\n");
			return;
		}
		if(sqlite3_open(db_info_path,&db_info)){
			printf("Error opening info db.\n");
			return;
		}
		setup_new_db();
	}
	else {
		if(sqlite3_open(db_path,&db)){
			printf("Error opening main db.\n");
			return;
		}
		if(sqlite3_open(db_backup_path,&db_backup)){
			printf("Error opening backup db.\n");
			return;
		}
		if(sqlite3_open(db_info_path,&db_info)){
			printf("Error opening info db.\n");
			return;
		}
		printf("opening existing.\n");
	}
}


/* 
 * adds an entry for environmental firefly data,
 * creating the table if necessary.
 */
void write_ff_env(struct firefly_env ff){
	char *dberror = NULL;
	char cmdbuf[BUFLEN];
  	int i;

	for(i=0; i<ERROR_RETRIES; i++ ){
		if(!table_exists(db, ff.id)){
			sprintf(cmdbuf,"create table '%s' (time int, light int, temp int, accl int, voltage int, audio int)",ff.id);
			sqlite3_exec(db,cmdbuf,callback,0,&dberror);
			sqlite3_exec(db_backup,cmdbuf,callback,0,&dberror);
			sprintf(cmdbuf,"create index '%s_index' on %s(time asc)", ff.id, ff.id);
			sqlite3_exec(db,cmdbuf,callback,0,&dberror);
			sqlite3_exec(db_backup,cmdbuf,callback,0,&dberror);
		}

		if(!device_exists(ff.id)){
			sprintf(cmdbuf,"insert into devices values ('%s', 'env', '%s')", ff.id, ff.id);
			sqlite3_exec(db_info,cmdbuf,callback,0,&dberror);
			sprintf(cmdbuf,"insert into default_group values('%s')", ff.id);
			sqlite3_exec(db_info,cmdbuf,callback,0,&dberror);
		}

		sprintf(cmdbuf,"insert into '%s' values(%d,",ff.id, ff.time);

		if(ff.light == INT_MIN) sprintf(cmdbuf,"%s NULL,",cmdbuf);
		else sprintf(cmdbuf,"%s %d,",cmdbuf, ff.light);

		if(ff.temp == INT_MIN) sprintf(cmdbuf,"%s NULL,",cmdbuf);
		else sprintf(cmdbuf,"%s %d,",cmdbuf, ff.temp);

		if(ff.accl == INT_MIN) sprintf(cmdbuf,"%s NULL,",cmdbuf);
		else sprintf(cmdbuf,"%s %d,",cmdbuf, ff.accl);

		if(ff.voltage == INT_MIN) sprintf(cmdbuf,"%s NULL,",cmdbuf);
		else sprintf(cmdbuf,"%s %d,",cmdbuf, ff.voltage);

		if(ff.audio == INT_MIN) sprintf(cmdbuf,"%s NULL)",cmdbuf);
		else sprintf(cmdbuf,"%s %d)",cmdbuf, ff.audio);

		sqlite3_exec(db,cmdbuf,callback,0,&dberror);
		sqlite3_exec(db_backup,cmdbuf,callback,0,&dberror);

		if(dberror)
		{
			printf("DB ERROR: %s\n",dberror);
			free(dberror);
			usleep(5000);
		} else return;
	}

}


/* 
 * adds an entry for power firefly (jigawatt) data,
 * creating the table if necessary.
 */
void write_power(struct power_meter pm ){
	char *dberror = NULL;
	char cmdbuf[BUFLEN];
  	int i;

	for(i=0; i<ERROR_RETRIES; i++ )
	{
		if(!table_exists(db, pm.id)){
			sprintf(cmdbuf,"create table '%s' (time int, state int, rms_current int, rms_voltage int, true_power int, energy int)",pm.id);
			sqlite3_exec(db,cmdbuf,callback,0,&dberror);
			sqlite3_exec(db_backup,cmdbuf,callback,0,&dberror);
			sprintf(cmdbuf,"create index '%s_index' on %s(time asc)", pm.id, pm.id);
			sqlite3_exec(db,cmdbuf,callback,0,&dberror);
			sqlite3_exec(db_backup,cmdbuf,callback,0,&dberror);
		}

		if(!device_exists(pm.id)){
			sprintf(cmdbuf,"insert into devices values ('%s', 'pow', '%s')", pm.id, pm.id);
			sqlite3_exec(db_info,cmdbuf,callback,0,&dberror);
			sprintf(cmdbuf,"insert into default_group values('%s')", pm.id);
			sqlite3_exec(db_info,cmdbuf,callback,0,&dberror);
		}

		sprintf(cmdbuf,"insert into '%s' values(%d,",pm.id,pm.time);

		if(pm.state == INT_MIN) sprintf(cmdbuf,"%s NULL,",cmdbuf);
		else sprintf(cmdbuf,"%s %d,",cmdbuf, pm.state);

		if(pm.rms_current == INT_MIN) sprintf(cmdbuf,"%s NULL,",cmdbuf);
		else sprintf(cmdbuf,"%s %d,",cmdbuf, pm.rms_current);

		if(pm.rms_voltage == INT_MIN) sprintf(cmdbuf,"%s NULL,",cmdbuf);
		else sprintf(cmdbuf,"%s %d,",cmdbuf, pm.rms_voltage);

		if(pm.true_power == INT_MIN) sprintf(cmdbuf,"%s NULL,",cmdbuf);
		else sprintf(cmdbuf,"%s %d,",cmdbuf, pm.true_power);

		if(pm.energy == INT_MIN) sprintf(cmdbuf,"%s NULL)",cmdbuf);
		else sprintf(cmdbuf,"%s %d)",cmdbuf, pm.energy);

		sqlite3_exec(db,cmdbuf,callback,0,&dberror);
		sqlite3_exec(db_backup,cmdbuf,callback,0,&dberror);

		if(dberror){
			printf("DB ERROR: %s\n",dberror);
			free(dberror);
			usleep(5000);
		} else return;
	}
}



/* 
 * adds an entry for generic integer data,
 * creating the table if necessary.
 */
void write_generic_integer(struct generic_integer_sensor gen){
	char *dberror = NULL;
	char cmdbuf[BUFLEN];
  	int i;

	for(i=0; i<ERROR_RETRIES; i++ )
	{
		if(!table_exists(db, gen.id)){
			sprintf(cmdbuf,"create table '%s' (time int, type char(16), value int)",gen.id);
			sqlite3_exec(db,cmdbuf,callback,0,&dberror);
			sqlite3_exec(db_backup,cmdbuf,callback,0,&dberror);
			sprintf(cmdbuf,"create index '%s_index' on %s(time asc)", gen.id, gen.id);
			sqlite3_exec(db,cmdbuf,callback,0,&dberror);
			sqlite3_exec(db_backup,cmdbuf,callback,0,&dberror);
		}

		if(!device_exists(gen.id)){
			sprintf(cmdbuf,"insert into devices values ('%s', 'gen', '%s')", gen.id, gen.id);
			sqlite3_exec(db_info,cmdbuf,callback,0,&dberror);
			sprintf(cmdbuf,"insert into default_group values('%s')", gen.id);
			sqlite3_exec(db_info,cmdbuf,callback,0,&dberror);
		}

		sprintf(cmdbuf,"insert into '%s' values(%d, '%s', %d)",gen.id, gen.time, gen.type, gen.value);

		sqlite3_exec(db,cmdbuf,callback,0,&dberror);
		sqlite3_exec(db_backup,cmdbuf,callback,0,&dberror);
	
		if(dberror)
		{
			printf("DB ERROR: %s\n",dberror);
			free(dberror);
			usleep(5000);
		} else return;

	}
}




/* 
 * adds an entry for statistical firefly data,
 * creating the table if necessary.
 */
void write_ff_stats(struct firefly_stats stats){
	char *dberror = NULL;
	char cmdbuf[BUFLEN];
  	int i;

	for(i=0; i<ERROR_RETRIES; i++ )
	{
		if(!table_exists(db, stats.id)){
			sprintf(cmdbuf,"create table '%s' (time int, tx int, rx int, uptime int, deep_sleep int, idle_time int, samples int)",stats.id);
			sqlite3_exec(db,cmdbuf,callback,0,&dberror);
			sqlite3_exec(db_backup,cmdbuf,callback,0,&dberror);
			sprintf(cmdbuf,"create index '%s_index' on %s(time asc)", stats.id, stats.id);
			sqlite3_exec(db,cmdbuf,callback,0,&dberror);
			sqlite3_exec(db_backup,cmdbuf,callback,0,&dberror);
		}

		if(!device_exists(stats.id)){	
			sprintf(cmdbuf,"insert into devices values ('%s', 'stats', '%s')", stats.id, stats.id);
			sqlite3_exec(db_info,cmdbuf,callback,0,&dberror);
						
			// sprintf(cmdbuf,"insert into default_group values('%s')", stats.id);
			// sqlite3_exec(db_info,cmdbuf,callback,0,&dberror);
		}

		sprintf(cmdbuf,"insert into '%s' values(%d,",stats.id, stats.time);

		if(stats.tx_pkts == INT_MIN) sprintf(cmdbuf,"%s NULL,",cmdbuf);
		else sprintf(cmdbuf,"%s %d,",cmdbuf, stats.tx_pkts);

		if(stats.rx_pkts == INT_MIN) sprintf(cmdbuf,"%s NULL,",cmdbuf);
		else sprintf(cmdbuf,"%s %d,",cmdbuf, stats.rx_pkts);

		if(stats.uptime == INT_MIN) sprintf(cmdbuf,"%s NULL,",cmdbuf);
		else sprintf(cmdbuf,"%s %d,",cmdbuf, stats.uptime);

		if(stats.deep_sleep == INT_MIN) sprintf(cmdbuf,"%s NULL,",cmdbuf);
		else sprintf(cmdbuf,"%s %d,",cmdbuf, stats.deep_sleep);

		if(stats.idle_time == INT_MIN) sprintf(cmdbuf,"%s NULL,",cmdbuf);
		else sprintf(cmdbuf,"%s %d,",cmdbuf, stats.idle_time);

		if(stats.sensor_samples == INT_MIN) sprintf(cmdbuf,"%s NULL)",cmdbuf);
		else sprintf(cmdbuf,"%s %d)",cmdbuf, stats.sensor_samples);

		sqlite3_exec(db,cmdbuf,callback,0,&dberror);
		sqlite3_exec(db_backup,cmdbuf,callback,0,&dberror);
	
		if(dberror)
		{
			printf("DB ERROR: %s\n",dberror);
			free(dberror);
			usleep(5000);
		} else return;

	}
}



/* 
 * adds an entry for location data,
 * creating the table if necessary.
 */
void write_location(struct location lc){
	char *dberror = NULL;
	char cmdbuf[BUFLEN];
  	int i;	
	for(i=0; i<ERROR_RETRIES; i++ )
	{
		if(!table_exists(db, lc.id)){
			sprintf(cmdbuf,"create table '%s' (time int, loc char(16))",lc.id);
			sqlite3_exec(db,cmdbuf,callback,0,&dberror);
			sqlite3_exec(db_backup,cmdbuf,callback,0,&dberror);
			sprintf(cmdbuf,"create index '%s_index' on %s(time asc)", lc.id, lc.id);
			sqlite3_exec(db,cmdbuf,callback,0,&dberror);
			sqlite3_exec(db_backup,cmdbuf,callback,0,&dberror);
		}

		if(!device_exists(lc.id)){		
			sprintf(cmdbuf,"insert into devices values ('%s', 'loc', '%s')", lc.id, lc.id);
			sqlite3_exec(db_info,cmdbuf,callback,0,&dberror);
			sprintf(cmdbuf,"insert into default_group values('%s')", lc.id);
			sqlite3_exec(db_info,cmdbuf,callback,0,&dberror);
		}

		sprintf(cmdbuf,"insert into '%s' values(%d, '%s')",lc.id, lc.time, lc.loc);

		sqlite3_exec(db,cmdbuf,callback,0,&dberror);
		sqlite3_exec(db_backup,cmdbuf,callback,0,&dberror);
	
		if(dberror)
		{
			printf("DB ERROR: %s\n",dberror);
			free(dberror);
			usleep(5000);
		} else return;

	}
}


/* 
 * adds an entry for neightbor list data,
 * creating the table if necessary.
 */
void write_neighbor_list(struct neighbor_list nl){
	char *dberror = NULL;
	char cmdbuf[BUFLEN];
  	int i;	

	for(i=0; i<ERROR_RETRIES; i++ )
	{
		if(!table_exists(db, nl.id)){
			sprintf(cmdbuf,"create table '%s' (time int, nbr char(16))",nl.id);
			sqlite3_exec(db,cmdbuf,callback,0,&dberror);
			sqlite3_exec(db_backup,cmdbuf,callback,0,&dberror);
			sprintf(cmdbuf,"create index '%s_index' on %s(time asc)", nl.id, nl.id);
			sqlite3_exec(db,cmdbuf,callback,0,&dberror);
			sqlite3_exec(db_backup,cmdbuf,callback,0,&dberror);
		}

		if(!device_exists(nl.id)){
			sprintf(cmdbuf,"insert into devices values ('%s', 'nlist', '%s')", nl.id, nl.id);
			sqlite3_exec(db_info,cmdbuf,callback,0,&dberror);
			sprintf(cmdbuf,"insert into default_group values('%s')", nl.id);
			sqlite3_exec(db_info,cmdbuf,callback,0,&dberror);
		}

		sprintf(cmdbuf,"insert into '%s' values(%d, '%s')",nl.id, nl.time, nl.neighbor);

		sqlite3_exec(db,cmdbuf,callback,0,&dberror);
		sqlite3_exec(db_backup,cmdbuf,callback,0,&dberror);
	
		if(dberror)
		{
			printf("DB ERROR: %s\n",dberror);
			free(dberror);
			usleep(5000);
		} else return;

	}
}



/* 
 * sets the alias of a device in the device table.
 */
void set_ff_alias(char *id, char *alias){
	char *dberror = NULL;	
	char cmdbuf[BUFLEN];

	int i;	

	for(i=0; i<ERROR_RETRIES; i++ )
	{
		sprintf(cmdbuf,"UPDATE devices SET alias='%s' WHERE id='%s'", alias, id);
		sqlite3_exec(db_info,cmdbuf,callback,0,&dberror);
	
		if(dberror)
		{
			printf("DB ERROR: %s\n",dberror);
			free(dberror);
			usleep(5000);
		} else return;
	}
}


/*
 *Retrieves the time of the last write of data.
 */
int get_last_write_time(){
	char cmdbuf[BUFLEN];
	char *dberror = NULL;
	char **tablenames, **curtime;
	int i, row, col, row2, col2;
	int maxtime = 0;


	sqlite3_get_table(db,"SELECT name FROM sqlite_master WHERE type='table' ORDER BY name",&tablenames,&row,&col,&dberror);
	
	if(row>0 && col>0){
		for(i=1; i<=row; i++){
			sprintf(cmdbuf,"select max(time) from '%s'", tablenames[i]);
			sqlite3_get_table(db,cmdbuf,&curtime,&row2,&col2,&dberror);
		
			if(curtime && curtime[1] && (maxtime < atoi(curtime[1])))
				maxtime = atoi(curtime[1]);
			sqlite3_free_table(curtime);
		}
	}
	
	if(dberror){
		printf("DB ERROR: %s\n",dberror);
		free(dberror);
		//usleep(5000);
	}
	sqlite3_free_table(tablenames);
	
	return maxtime;
}


/**** GROUP EDITING FUNCTIONS ****/



/* 
 * creates group with name and type inside of groups table.
 */
void create_group(char *group_name, char *group_type){
	char cmdbuf[64];
	char *dberror = NULL;
 	int i;	

	for(i=0; i<ERROR_RETRIES; i++ )
	{
		if(!device_exists(group_name)){	
			sprintf(cmdbuf,"create table '%s' (id char(32))", group_name);
			sqlite3_exec(db_info,cmdbuf,NULL,0,&dberror);

			sprintf(cmdbuf,"insert into groups values('%s','%s', NULL)", group_name, group_type);
			sqlite3_exec(db_info,cmdbuf,NULL,0,&dberror);
		}

		if(dberror)
		{
			printf("%s\n",dberror);
			free(dberror);
			usleep(5000);
		} else return;
	}
}


/* 
 * removes selected group.
 */
void remove_group(char *group_name){
	char cmdbuf[128];
	char *dberror = NULL;
	char **result;
	int row, col;
  	int i;	

	for(i=0; i<ERROR_RETRIES; i++ )
	{
		sprintf(cmdbuf,"select * from '%s'", group_name);
		sqlite3_get_table(db_info,cmdbuf,&result,&row,&col,&dberror);
		sqlite3_free_table(result);
	
		if(row==0 && col==0){
			sprintf(cmdbuf,"drop table '%s'", group_name);
			sqlite3_exec(db_info,cmdbuf,NULL,0,&dberror);

			sprintf(cmdbuf,"delete from groups where group_name='%s'", group_name);
			sqlite3_exec(db_info,cmdbuf,NULL,0,&dberror);
		}
	
		if(dberror)
		{
			printf("%s\n",dberror);
			free(dberror);
			usleep(5000);
		} else return;
	}

}


/* 
 * add device to group table with name 'group_name'. 
 */
void add_to_group(char *id, char *group_name){
	char cmdbuf[BUFLEN];
	char *dberror = NULL;
	char **result;
	int row, col;
  	int i;

	for(i=0; i<ERROR_RETRIES; i++ )
	{
		sprintf(cmdbuf,"select id from '%s' where id='%s'",group_name,id);
		sqlite3_get_table(db_info,cmdbuf,&result,&row,&col,&dberror);
		sqlite3_free_table(result);

		if(row==0 && col==0){
			sprintf(cmdbuf,"select group_name from groups where group_name='%s' and group_type=(select type from devices where id='%s')", group_name, id);
			sqlite3_get_table(db_info,cmdbuf,&result,&row,&col,&dberror);
			sqlite3_free_table(result);
			if(row!=0 && col!=0){
				sprintf(cmdbuf,"insert into '%s' values('%s')", group_name, id);
				sqlite3_exec(db_info,cmdbuf,NULL,0,&dberror);
				sprintf(cmdbuf,"delete from default_group where id='%s'", id);
				sqlite3_exec(db_info,cmdbuf,NULL,0,&dberror);
			}
		}

		if(dberror)
		{
			printf("DB ERROR: %s\n",dberror);
			free(dberror);
			usleep(5000);
		} else return;

	}

}


/* 
 * removes device from group table with name 'group_name'.
 * 'group_name' must always be specified because devices 
 * can be members of multiple groups.
 */
void remove_from_group(char *id, char *group_name){
	char cmdbuf[BUFLEN];
	char *dberror = NULL;
	char **result;
	char **result2;
	int i, row, col, row2, col2;

	for(i=0; i<ERROR_RETRIES; i++ )
	{
		sprintf(cmdbuf,"delete from '%s' where id='%s'",group_name,id);
		sqlite3_exec(db_info,cmdbuf,NULL,0,&dberror);
		sprintf(cmdbuf,"insert into default_group values('%s')", id);
		sqlite3_exec(db_info,cmdbuf,NULL,0,&dberror);

		sqlite3_get_table(db_info,"select group_name from groups",&result,&row,&col,&dberror);

		for(i=1; i<=row; i++){
			sprintf(cmdbuf,"select id from '%s' where id='%s'",result[i],id);
			sqlite3_get_table(db_info,cmdbuf,&result2,&row2,&col2,&dberror);
			sqlite3_free_table(result2);
			if(row2!=0 && col2!=0){
				sprintf(cmdbuf,"delete from default_group where id='%s'", id);
				sqlite3_exec(db_info,cmdbuf,NULL,0,&dberror);
				break;
			}
		}
		sqlite3_free_table(result);
	
		if(dberror)
		{
			printf("DB ERROR: %s\n",dberror);
			free(dberror);
			usleep(5000);
		} else return;
	}

}



/* 
 * sets new name of a group.
 */
void rename_group(char *old_group_name, char *new_group_name){
	char *dberror = NULL;	
	char cmdbuf[BUFLEN];
  	int i;

	for(i=0; i<ERROR_RETRIES; i++ )
	{
		sprintf(cmdbuf,"UPDATE groups SET group_name='%s' WHERE group_name='%s'", new_group_name, old_group_name);
		sqlite3_exec(db_info,cmdbuf,callback,0,&dberror);
		sprintf(cmdbuf,"alter table '%s' rename to '%s'", old_group_name, new_group_name);
		sqlite3_exec(db_info,cmdbuf,callback,0,&dberror);

		if(dberror)
		{
			printf("DB ERROR: %s\n",dberror);
			free(dberror);
			usleep(5000);
		} else return;


	}

}



/**** HELPER FUNCTIONS ****/


/* 
 * prepares an existing empty database to receive data from devices.
 * helper function for init_db().
 */
void setup_new_db(){
	char *dberror = NULL;
  	int i;

	printf("setting up new db\n");

	for(i=0; i<ERROR_RETRIES; i++ ){
		sqlite3_exec(db_info,"create table devices (id char(32), type char(16), alias char(64))",NULL,0,&dberror);
		sqlite3_exec(db_info,"create table groups (group_name char(64), group_type char(16), master_device char(32))",NULL,0,&dberror);
		sqlite3_exec(db_info,"create table default_group (id char(32))",NULL,0,&dberror);
	

		if(dberror)
		{
			printf("DB ERROR: %s\n",dberror);
			free(dberror);
			usleep(5000);
		} 
		else 
			return;
	}

	return;
}



/* 
 * prints the elements returned from a call to sqlite3_get_table().
 * useful for debugging.
 */
void print_table(char **result, int rownum, int colnum){
	int i, j;

	for(i=0; i<=rownum; i++){
		for(j=0; j<colnum; j++){
			printf("%s\t",result[(i*colnum)+j]);
		}
		printf("\n");
	}

	return;
}



/* 
 * returns 1 if a table with name 'table_name' exists
 * in the database, returns 0 otherwise. 
 */
int table_exists(sqlite3 *db_name, char *table_name){
	char cmdbuf[128];
	char **result;
	int row, col, table_exists;

	sprintf(cmdbuf,"select min(rowid) from '%s'",table_name);
	table_exists = !(sqlite3_get_table(db_name,cmdbuf,&result,&row,&col,NULL));
	sqlite3_free_table(result);
	return table_exists;
}


/* 
 * returns 1 if a table with name 'table_name' exists
 * in the database, returns 0 otherwise. 
 */
int device_exists (char *table_name){
	char cmdbuf[128];
	char **result;
	int row, col, dev_exists;

	sprintf(cmdbuf,"select * from 'devices' where id='%s'",table_name);
	sqlite3_get_table(db_info,cmdbuf,&result,&row,&col,NULL);
	sqlite3_free_table(result);
	dev_exists = (row==0 && col==0) ? 0 : 1;

	return dev_exists;
}

/* 
 * is called by sqlite3_exec() to print db tables or elements.
 * use sqlite3_get_table() as an alternative if you wish to retrieve 
 * data, as opposed to just printing it.
 */
static int callback(void *NotUsed, int argc, char **argv, char **azColName){
  int i;
  for(i=0; i<argc; i++)
    printf("%s = %s\n", azColName[i], argv[i] ? argv[i] : "NULL");
  printf("\n");
  return 0;
}



/* 
 * builds mock fireflies for testing 
 */
void build_ffs(struct firefly_env *ff_env, struct power_meter *pm){
	ff_env->id = malloc(sizeof(char)*16);
	strcpy(ff_env->id,"ff_00000002");
	ff_env->time  = time(NULL);
	ff_env->light = SMALL_RAND;
	ff_env->temp = SMALL_RAND;
	ff_env->accl = SMALL_RAND;
	ff_env->voltage = SMALL_RAND;
	ff_env->audio = SMALL_RAND;

	pm->id = malloc(sizeof(char)*16);
	strcpy(pm->id,"ff_000000f0_0");
	pm->time  = time(NULL);
	pm->state = SMALL_RAND;
	pm->rms_current = SMALL_RAND;
	pm->rms_voltage = SMALL_RAND;
	pm->true_power = SMALL_RAND;
	pm->energy = SMALL_RAND;

	return;
}


/* 
 * increment mock fireflies
 */
void increment_ffs(struct firefly_env *ff_env, struct power_meter *pm){
	ff_env->time = time(NULL);
	ff_env->light += 2;
	ff_env->temp += 4;
	ff_env->accl += 8;
	ff_env->voltage += 16;
	ff_env->audio += 32;

	pm->time = time(NULL);
	pm->state += 2;
	pm->rms_current += 4;
	pm->rms_voltage += 8;
	pm->true_power += 16;
	pm->energy += 32;

	return;
}



#endif
