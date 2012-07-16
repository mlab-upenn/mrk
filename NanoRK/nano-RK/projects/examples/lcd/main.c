#include <nrk.h>
#include <include.h>
#include <ulib.h>
#include <stdio.h>
#include <avr/sleep.h>
#include <hal.h>
#include <nrk_error.h>
#include <nrk_timer.h>
#include <ff_lcd.h>
#include <lcd_driver.h>

NRK_STK Stack1[NRK_APP_STACKSIZE];
nrk_task_type TaskOne;
void Task1(void);

void nrk_create_taskset();

int main()
{
  nrk_setup_ports();
  nrk_setup_uart(UART_BAUDRATE_115K2);

  nrk_init();

  nrk_led_clr(0);
  nrk_led_clr(1);
  nrk_led_clr(2);
  nrk_led_clr(3);

  nrk_time_set(0, 0);

  nrk_create_taskset ();
  nrk_start();

  return 0;
}

void Task1()
{
  uint8_t secret = 0;
  char* s = "Screen 1\\nWelcome!\\nScreen 2\\nDriver\\r\\nScreen 3\\n \\dTest\\d";
  char* s_secret = "Congrats\\ndude\\nYou\\nfound it\\nSecret\\ncode!";

  // Initialize FireFly LCD v1.2 board
  lcd_setup();

  // Set LEDs
  lcd_led_set(1, 0);
  lcd_led_set(2, 0);
  lcd_led_set(3, 1);

  lcd_string_display_escape("FireFly\\nLCD v1.2");
  lcd_wait_us(2000000); // Wait 2 seconds

  lcd_string_display_escape("Basic\\nLCD");
  lcd_wait_us(2000000); // Wait 2 seconds

  lcd_string_display_escape("Driver\\nTest");
  lcd_wait_us(2000000); // Wait 2 seconds

  // Load initial string array
  lcd_string_array_load(s);

  // Switch input loop
  while(1)
  {
    if(lcd_switch_pressed(1)) // If switch 1 is pressed
    {
      if(!secret)
        lcd_led_set(1, 1); // Set LED 1

      // Buzz
      lcd_buzz(30);

      if(lcd_switch_pressed(2)) // If both switches are pressed
      {
        // Load new string array
        lcd_string_array_load(s_secret);

        // Set LEDs
        lcd_led_set(1, 1);
        lcd_led_set(2, 1);
        lcd_led_set(3, 1);

        secret = 1;
      }
      else
      {
        // Cycle string array
        lcd_string_array_cycle();
      }
    }
    else if(lcd_switch_pressed(2)) // If switch 2 is pressed
    {
      if(!secret)
        lcd_led_set(2, 1); // Set LED 2

      // Buzz
      lcd_buzz(30);

      // Reverse cycle string array
      lcd_string_array_cycle_reverse();
    }
    else if(!secret) // If no switches pressed
    {
      lcd_led_set(1, 0); // Clear LED 1
      lcd_led_set(2, 0); // Clear LED 2
    }

    if(secret)
    {
      // Toggle LEDs
      lcd_led_toggle(1);
      lcd_led_toggle(2);
      lcd_led_toggle(3);
    }

    nrk_wait_until_next_period();
  }
}

void nrk_create_taskset()
{
  TaskOne.task = Task1;
  nrk_task_set_stk(&TaskOne, Stack1, NRK_APP_STACKSIZE);
  TaskOne.prio = 1;
  TaskOne.FirstActivation = TRUE;
  TaskOne.Type = BASIC_TASK;
  TaskOne.SchType = PREEMPTIVE;
  TaskOne.period.secs = 0;
  TaskOne.period.nano_secs = 100 * NANOS_PER_MS;
  TaskOne.cpu_reserve.secs = 1;
  TaskOne.cpu_reserve.nano_secs = 50 * NANOS_PER_MS;
  TaskOne.offset.secs = 0;
  TaskOne.offset.nano_secs = 0;
  nrk_activate_task(&TaskOne);
}
