#ifndef TDMA_ASAP_STATS_H
#define TDMA_ASAP_STATS_H

#include "tdma_asap.h"

#ifdef NRK_STATS_TRACKER
#include <nrk_stats.h>
nrk_task_stat_t tdma_stat_struct;
#endif

nrk_time_t tdma_cpu_time;

// time needed to create the tree; we don't want to use this
// to factor into tdma's energy
nrk_time_t tree_creation_time;
nrk_time_t osa_time;

uint8_t tdma_pid;
uint8_t rdo_running;
uint32_t ticks_rdo;

// histogram of missing 1-10 sync packets in a row
#ifdef SYNC_HIST
uint16_t sync_hist[10];
uint8_t sync_consec_miss;
#endif

// log of mpkt sends and receives to be printed out periodically
#ifdef MPKT_LOG
char mpkt_log[25][35];
uint8_t mpkt_log_curbuf;
#endif
/*
void _tdma_timer4_reset();
void _tdma_timer4_stop();
void _tdma_timer4_setup();
inline uint16_t _tdma_timer4_get();

void _tdma_timer5_reset();
void _tdma_timer5_stop();
void _tdma_timer5_setup();
inline uint16_t _tdma_timer5_get();
*/

/*
uint8_t steal_rdo_running;
uint8_t reg_rdo_running;

uint8_t recording_awk_reg;
uint8_t recording_awk_steal;

uint32_t reg_micros_awk;
uint32_t steal_micros_awk;

uint32_t reg_micros_rdo; 
uint32_t steal_micros_rdo; 
*/

uint32_t steal_cnt; 
uint32_t backoff_cnt;
uint32_t dbl_pkt_cnt;
// # sync packets rcvd (100% if equal to sync_slot_cnt)
uint16_t sync_rx_cnt;
// # of sync slots
uint16_t sync_slot_cnt;

/*
void steal_cca_check();
void steal_rdo_start();
void steal_rdo_stop();
void reg_rdo_start();
void reg_rdo_stop();
*/
void sync_miss_log();
void sync_rx_log();

void steal_log();
void steal_backoff_log();
void tdma_stats_setup(uint8_t pid);
void tdma_stats_dump();

void tdma_stats_record_tree_creation();

void tdma_stats_add_osa(nrk_time_t t);

// timestamp the packet
void tdma_stats_ts_packet(uint8_t * tx_buf);

/*
void stats_begin_awk_steal();
void stats_end_awk_steal();
void stats_begin_awk_reg();
void stats_end_awk_reg();

void log_awake();
void log_sleep();
void log_rdo_on();
void log_rdo_off();
*/
#endif
