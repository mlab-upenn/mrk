#include <globals.h>
#include <nrk.h>
#include <sampl.h>
#include <transducer_handler.h>
#include <transducer_registry.h>
#include <lcd_driver.h>

// This is a callback function that gets executed when a transducer
// packet addressed to this node arrives.  Messages packed into out_msg
// are then returned.
int8_t transducer_handler(TRANSDUCER_MSG_T *in_msg, TRANSDUCER_MSG_T *out_msg)
{
  nrk_kprintf(PSTR("in_msg\r\n  mac_addr = 0x"));
  printf("%x\r\n", in_msg->mac_addr);

  nrk_kprintf(PSTR("  type = "));
  printf("%d\r\n", in_msg->type);

  nrk_kprintf(PSTR("  len = "));
  printf("%d\r\n", in_msg->len);

  printf("  payload = \"%s\"\r\n", (char*)(in_msg->payload));

  // By default nothing will be returned because you don't want to NCK back to packets for other devices
  // Make sure to return 0 if you did not add a packet and 1 if you did.

  // Return ACK if good and no other reply
  // out_msg->type=TRAN_ACK;
  // out_msg->len=0;
  // return 1;

  // Return NCK if bad and no other reply
  // out_msg->type=TRAN_NCK;
  // out_msg->len=0;
  // return 1;

  // If LCD message, load and show on screen
  if(in_msg->type == TRAN_LCD_MESSAGE)
  {
    nrk_led_set(GREEN_LED);
    lcd_string_array_load(in_msg->payload);
    nrk_led_clr(GREEN_LED);

    out_msg->type = TRAN_ACK;
    out_msg->len = 0;
    return 1;
  }

  return 0;
}
