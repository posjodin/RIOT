#define VERSION "0.5 2021-01-19" /* Robert Olsson/KTH 20210-01-13 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include "board.h"
#include "shell.h"
#include "thread.h"
#include "msg.h"
#include "periph/rtt.h"
#include "periph/i2c.h"
#include <errno.h>
#include <stdint.h>
#include "xtimer.h"
#include "pms5003.h"

#ifndef SHELL_BUFSIZE
#define SHELL_BUFSIZE       (128U)
#endif

char line_buf[SHELL_BUFSIZE];

static int cmd_ver(__attribute__((unused)) int ac, __attribute__((unused)) char **av)
{
  printf("%s\n", VERSION);
  return 0;
}

static int cmd_read(__attribute__((unused)) int ac, __attribute__((unused)) char **av)
{
  printf("PM[1 2.5 10]: %-u %-u %-u\n", pms5003_pm1(), pms5003_pm2_5(), pms5003_pm10());
  printf("DB[0.3 0.5 1.0 2.5 5 10]: %-u %-u %-u %-u %-u %-u\n", pms5003_db0_3(), pms5003_db0_5(),
	 pms5003_db1(),  pms5003_db2_5(), pms5003_db5(), pms5003_db10());
  return 0;
}

static const shell_command_t shell_commands[] = {
  { "ver", "version", cmd_ver},
  { "read", "read PM sensor data", cmd_read},
  { NULL, NULL, NULL }
};

int main(void)
{
  pms5003_init();
  shell_run(shell_commands, line_buf, SHELL_BUFSIZE);
  return 0;
}
