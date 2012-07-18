#include <stdio.h>
#include <stdlib.h>
#include <error_log.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

static uint8_t log_enabled;
static char timeStr[64];
static time_t timestamp;
static FILE *log;

void log_init ()
{
  log_enabled = 1;
  log = fopen ("logs/system.log", "a");
  if (log == NULL) {
    printf ("Error, could not open logs/system.log!\n");
    exit (0);
  }
  log_write ("System Starting Up");

}

void log_write (char *msg)
{
  if (log_enabled) {
    time (&timestamp);
    strftime (timeStr, 100, "%Y-%m-%d %X", localtime (&timestamp));
    fprintf (log, "%s: %s\n", timeStr, msg);
    fflush (log);
  }
}
