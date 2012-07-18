#include <globals.h>
#include <nrk.h>
#include <sampl.h>
#include <transducer_pkt.h>

#define socket_0_enable()       nrk_gpio_set(NRK_DEBUG_0)
#define socket_1_enable()       nrk_gpio_set(NRK_DEBUG_1)
#define socket_0_disable()      nrk_gpio_clr(NRK_DEBUG_0)
#define socket_1_disable()      nrk_gpio_clr(NRK_DEBUG_1)


#define power_mon_enable()       nrk_gpio_clr(NRK_DEBUG_2)
#define power_mon_disable()      nrk_gpio_set(NRK_DEBUG_2)


void energy_eeprom_read(void);
void energy_eeprom_write(void);
void energy_eeprom_erase(void);
int8_t transducer_handler(TRANSDUCER_MSG_T *in_msg, TRANSDUCER_MSG_T *out_msg);
