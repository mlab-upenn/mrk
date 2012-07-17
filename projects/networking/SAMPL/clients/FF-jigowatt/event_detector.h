#ifndef EVENT_DETECTOR_H_
#define EVENT_DETECTOR_H_

#include <nrk.h>

NRK_STK event_detector_stack[256];
nrk_task_type event_detector;
void event_detector_task();
nrk_sig_t update_energy_sig;

#endif
