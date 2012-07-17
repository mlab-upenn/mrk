/******************************************************************************
*  Nano-RK, a real-time operating system for sensor networks.
*  Copyright (C) 2007, Real-Time and Multimedia Lab, Carnegie Mellon University
*  All rights reserved.
*
*  This is the Open Source Version of Nano-RK included as part of a Dual
*  Licensing Model. If you are unsure which license to use please refer to:
*  http://www.nanork.org/nano-RK/wiki/Licensing
*
*  This program is free software: you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation, version 2.0 of the License.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*
*  You should have received a copy of the GNU General Public License
*  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*******************************************************************************/

#include <nrk.h>
#include <include.h>
#include <ulib.h>
#include <stdio.h>
#include <hal.h>
#include <nrk_error.h>
#include <nrk_timer.h>
#include <nrk_stack_check.h>
#include <nrk_stats.h>


NRK_STK Stack1[NRK_APP_STACKSIZE];
nrk_task_type TaskOne;
void Task1(void);


void nrk_create_taskset();

int
main ()
{
    nrk_setup_ports();
    nrk_setup_uart(UART_BAUDRATE_115K2);

    nrk_init();

    nrk_led_clr(ORANGE_LED);
    nrk_led_clr(BLUE_LED);
    nrk_led_clr(GREEN_LED);
    nrk_led_clr(RED_LED);

    nrk_time_set(0,0);
    nrk_create_taskset ();
    nrk_start();

    return 0;
}
    nrk_time_t start_time_1, start_time, end_time, end_time_1;

void Task1()
{
    nrk_kprintf( PSTR("Nano-RK Version ") );
    printf( "%d\r\n",NRK_VERSION );


    start_time.secs=0;
    start_time.nano_secs=0;
    end_time.secs=0;
    end_time.nano_secs=0;
    start_time_1.secs=0;
    start_time_1.nano_secs=0;
    end_time_1.secs=0;
    end_time_1.nano_secs=0;

    while (1) {

        // Measure time with code inbetween
        nrk_time_get(&start_time);
        nrk_time_get(&end_time);

        printf("####### NO WAIT BETWEEN READINGS #########\r\n");
        printf("start: %lu %lu\r\n",start_time.secs,start_time.nano_secs);
        printf("  end: %lu %lu\r\n",end_time.secs,end_time.nano_secs);
        printf("end.nano - start.nano: %lu\r\n",end_time.nano_secs-start_time.nano_secs);
        //printf("start.nano - end.nano: %lu\r\n",start_time.nano_secs-end_time.nano_secs);

        // Measure time with code inbetween
        nrk_time_get(&start_time_1);
        nrk_spin_wait_us(200);
        nrk_time_get(&end_time_1);


        printf("####### 200us BETWEEN READINGS #########\r\n");
        printf("start: %lu %lu\r\n",start_time_1.secs,start_time_1.nano_secs);
        printf("  end: %lu %lu\r\n",end_time_1.secs,end_time_1.nano_secs);
        printf("end.nano - start.nano: %lu\r\n",end_time_1.nano_secs-start_time_1.nano_secs);

        nrk_wait_until_next_period();
    }

}

void
nrk_create_taskset()
{
    nrk_task_set_entry_function( &TaskOne, Task1);
    nrk_task_set_stk( &TaskOne, Stack1, NRK_APP_STACKSIZE);
    TaskOne.prio = 1;
    TaskOne.FirstActivation = TRUE;
    TaskOne.Type = BASIC_TASK;
    TaskOne.SchType = PREEMPTIVE;
    TaskOne.period.secs = 5;
    TaskOne.period.nano_secs = 250*NANOS_PER_MS;
    TaskOne.cpu_reserve.secs = 1;
    TaskOne.cpu_reserve.nano_secs = 50*NANOS_PER_MS;
    TaskOne.offset.secs = 0;
    TaskOne.offset.nano_secs= 0;
    nrk_activate_task (&TaskOne);
}

