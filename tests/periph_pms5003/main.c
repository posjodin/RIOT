#define VERSION "0.5 2021-01-18" /* Robert Olsson/KTH 20210-01-13 */
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

#define PMSMAXBODYLEN 28
#define PMSBUFFER (PMSMAXBODYLEN + 4)
extern uint8_t buf[PMSBUFFER];
extern int pmsframe(uint8_t *buf);
extern void printpm(void);
extern void printpm_old(void);

#define PMS5003_I2C_ADDR 0x12
#define PARSER_PRIO        (THREAD_PRIORITY_MAIN - 1)
#define PARSER_TYPE        (0xabcd)
#define QSIZE 32

#ifndef SHELL_BUFSIZE
#define SHELL_BUFSIZE       (128U)
#endif

uint16_t i2c_addr = PMS5003_I2C_ADDR;
i2c_t i2c_dev = 0;


char line_buf[SHELL_BUFSIZE];

static kernel_pid_t parser_pid;
static char parser_stack[THREAD_STACKSIZE_MAIN];

msg_t msg_queue[QSIZE];
//static msg_t msg;

int i2c_probe(i2c_t dev, uint16_t i2c_addr)
{
    char dummy[1];
    int retval;

    if (i2c_acquire(dev)){
        puts("Failed to acquire I2C device");
        return -1;
    }

    while (-EAGAIN == (retval = i2c_read_byte(dev, i2c_addr, dummy, 0))) {
      /* retry until bus arbitration succeeds */
    }

    switch (retval) {

    case 0:
      /* success: Device did respond */
      printf("Found 0x%02X\n", i2c_addr);
      break;
      
    case -ENXIO:
      /* No ACK --> no device */
      puts("Not found\n");
      break;
    default:
      /* Some unexpected error */
      puts("Err 2\n");
      break;
    }

    i2c_release(dev);
    return 0;
}

struct {
  uint16_t dist, signal;
  int temp;
  time_t time;
} lr;

void wait_ms(uint32_t timeout)
{
  uint32_t tmo = timeout * 1000U;
  xtimer_usleep(tmo);
}

inline uint32_t now(void)
{
  return rtt_get_counter();
}

int read_pms5002(i2c_t i2c_dev, uint16_t i2c_addr)
{
  int res = 1;
  uint16_t reg = 0;
  uint8_t flags = 0;

  res = i2c_acquire(i2c_dev);
  
  if (res) {
    return res;
  }
  
  res = i2c_read_regs(i2c_dev, i2c_addr, reg, buf, PMSBUFFER, flags);

  if(res)
    printf("res=%d\n", res);

  i2c_release(i2c_dev);

  pmsframe(buf);
  printpm();
  
  return res;
}

static void *parser(void *arg)
{
  (void)arg;

  while (1) {
    wait_ms(10*1000);
    read_pms5002(i2c_dev, i2c_addr);
  }
  return NULL;
}

static int cmd_ver(__attribute__((unused)) int ac, __attribute__((unused)) char **av)
{
  printf("%s\n", VERSION);
  return 0;
}

static int cmd_read(__attribute__((unused)) int ac, __attribute__((unused)) char **av)
{
  read_pms5002(i2c_dev, i2c_addr);
  return 0;
}

static const shell_command_t shell_commands[] = {
  { "ver", "code version", cmd_ver },
  { "read", "Run RFid inventory", cmd_read },
  { NULL, NULL, NULL }
};

int main(void)
{
  i2c_probe(i2c_dev, i2c_addr);

  /* start the parser thread */
  parser_pid = thread_create(parser_stack, sizeof(parser_stack),
			      PARSER_PRIO, 0, parser, NULL, "parser");

  shell_run(shell_commands, line_buf, SHELL_BUFSIZE);
  return 0;
}
