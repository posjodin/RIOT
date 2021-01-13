#define VERSION "0.91 2021-01-13" /* Robert Olsson/KTH 20210-01-13 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include "board.h"
#include "shell.h"
#include "thread.h"
#include "msg.h"
#include "ringbuffer.h"
#include "periph/uart.h"
#include "periph/rtt.h"
#include "xtimer.h"

#ifdef MODULE_STDIO_UART
#include "stdio_uart.h"
#endif

/* Command for tf01 plus */
#define  CMD_FW_VERSION 1
#define  CMD_RESET      2
#define  CMD_RATE       3
#define  CMD_TRIGGER    4
#define  CMD_OUT_FORMAT    5 /* Output format */
#define  CMD_BAUD       6
#define  CMD_OUT_DISABLE   7
#define  CMD_COMM_MODE  0xA  /* UART or I2C */
#define  CMD_I2C_ADDR   0xB  /* Set I2C ADDR */
#define  CMD_I2C_READ     0  /* cm or mm */
#define  CMD_IO_MODE   0x3B
#define  CMD_RESTORE_FACTORY   0x10
#define  CMD_SAVE_SETTING      0x11


#ifndef SHELL_BUFSIZE
#define SHELL_BUFSIZE       (128U)
#endif
#ifndef UART_BUFSIZE
#define UART_BUFSIZE        (128U)
#endif

#define PARSER_PRIO        (THREAD_PRIORITY_MAIN - 1)
#define PARSER_TYPE        (0xabcd)

#ifndef STDIO_UART_DEV
#define STDIO_UART_DEV      (UART_UNDEF)
#endif

typedef struct {
  char rx_mem[UART_BUFSIZE];
  ringbuffer_t rx_buf;
} uart_ctx_t;

struct {
  uint16_t dist, signal;
  int temp;
  time_t time;
} lr;

static uart_ctx_t ctx[UART_NUMOF];
static kernel_pid_t parser_pid;
static char parser_stack[THREAD_STACKSIZE_MAIN];
unsigned char buf[40];

void wait_ms(int timeout)
{
  uint32_t tmo = timeout * 1000U;
  xtimer_usleep(tmo);
}

inline uint32_t now(void)
{
  return rtt_get_counter();
}

int cmd_get_fw_version(int dev)
{
  buf[0] = 0x5A;
  buf[1] = 0x04;
  buf[2] = CMD_FW_VERSION;
  buf[3] = 0x5F;
  uart_write(UART_DEV(dev), buf, 4);
  return 1;
}

int cmd_reset(int dev)
{
  buf[0] = 0x5A;
  buf[1] = 0x04;
  buf[2] = CMD_RESET;
  buf[3] = 0x60;
  uart_write(UART_DEV(dev), buf, 4);
  return 1;
}

int cmd_set_rate(int dev, uint16_t rate)
{
  buf[0] = 0x5A;
  buf[1] = 0x06;
  buf[2] = CMD_RATE;
  buf[3] = rate & 0xFF;
  buf[4] = (rate << 8);
  buf[5] = buf[0] + buf[1] + buf[3] + buf[4];
  uart_write(UART_DEV(dev), buf, 6);
  return 1;
}

int cmd_trigger(int dev)
{
  buf[0] = 0x5A;
  buf[1] = 0x04;
  buf[2] = CMD_TRIGGER;
  buf[3] = 0x62;
  uart_write(UART_DEV(dev), buf, 4);
  return 1;
}

int cmd_restore_factory(int dev)
{
  buf[0] = 0x5A;
  buf[1] = 0x04; 
  buf[2]= CMD_RESTORE_FACTORY;
  buf[3] = 0x6E;
  uart_write(UART_DEV(dev), buf, 4);
  return 1;
}

int cmd_save_setting(int dev)
{
  buf[0] = 0x5A;
  buf[1] = 0x04; 
  buf[2]= CMD_SAVE_SETTING;
  buf[3] = 0x6F;
  uart_write(UART_DEV(dev), buf, 4);
  return 1;
}

#define QSIZE 32
msg_t msg_queue[QSIZE];
msg_t msg;

/* From IRQ context */

static void rx_cb(void *arg, uint8_t data)
{
  uart_t dev = (uart_t)arg;
  
  if(dev == 1) {
    ringbuffer_add_one(&(ctx[dev].rx_buf), data);
    if (data == 0x59 || data == 0x5A) {
      msg.content.value = (uint32_t) 1; // MSG_MESSAGE;
      if(msg_try_send(&msg, parser_pid) == 0) {
	puts("Queue full\n");
	msg_init_queue(msg_queue, QSIZE);
      }
    }
  }
}

unsigned char csum(unsigned char *in, unsigned char len) 
{
  int i;
  unsigned char cs = 0;
  for(i=0; i<len; i++) {
    cs += in[i];
  }
  return cs;
}

void parse_data(unsigned char *buf)
{  
  //printf("CS frame-cs=%02x csum=%02x\n", buf[8], csum(buf, 8));
  if(buf[8] != csum(buf, 8)) {
    return; // csum err
  }
  
  lr.dist = buf[2] + buf[3]*256;
  lr.signal = buf[4] + buf[5]*256;
  lr.temp = buf[6] + buf[7]*256;
  lr.temp = lr.temp/8 -256;
  lr.time = now();
  if(1) {
    printf("Dist=%-5u Signal=%-5u Temp=%-d", lr.dist, lr.signal, lr.temp);
    printf("\r");
  }
#if DEBUG
  int i;
  printf("parse_data\n");
  for(i=0; i < 9; i++)
    printf(" %02x", buf[i]);
  printf("\n");
#endif
}

void parse_command(unsigned char *buf)
{
  int i;
  uint16_t rate;
  uint8_t len = buf[1];
  uint8_t cmd = buf[2];
  
  switch(cmd) {

  case CMD_RESET:
    printf("RESET %s\n" ,buf[3]?"FAIL":"OK");
    break;

  case CMD_FW_VERSION:
    printf("Version V%-u.%-u.%-u\n", buf[5], buf[4], buf[3]);
    break;

  case CMD_RATE:
    rate = (((uint16_t) buf[4])<<8) + buf[3];
    printf("Rate %-u Hz\n", rate);
    break;

  case CMD_RESTORE_FACTORY:
    printf("RESTORE %s\n" ,buf[3]?"FAIL":"OK");
    break;

  case CMD_SAVE_SETTING:
    printf("SAVE SETTING %s\n" ,buf[3]?"FAIL":"OK");
    break;
    
  default:
    printf("parse_command: ");
    for(i=0; i < len; i++)
      printf(" %02x", buf[i]);
    printf("\n");

    break;
  }
}

static void *parser(void *arg)
{
  (void)arg;
  msg_t msg;
  int i;
  msg_init_queue(msg_queue, QSIZE);

  while (1) {
    msg_receive(&msg);
    uart_t dev = (uart_t)msg.content.value;
	
    if(dev != 1)
      continue;

    i = 0;
    while (! ringbuffer_empty(&(ctx[dev].rx_buf))) {
      buf[i++] = (int)ringbuffer_get_one(&(ctx[dev].rx_buf));
      if(i == (sizeof(buf)-1))
	break;
      xtimer_usleep(100); // Investigation needed */
    }

    if(buf[0] == 0x59 && buf[1] == 0x59) {
      parse_data(buf);
    }
    if(buf[0] == 0x5A) {
      parse_command(buf);
    }
  } 
  return NULL;
}

static int init_serial(int dev, uint32_t baud)
{
  int res;
    
  if (dev < 0) {
    return 1;
  }

  res = uart_init(UART_DEV(dev), baud, rx_cb, (void *)dev);
  if (res == UART_NOBAUD) {
    printf("Error: baudrate (%u)\n", (unsigned int)baud);
    return 1;
  }
  else if (res != UART_OK) {
    puts("Error: init UART\n");
    return 1;
  }
  printf("UART_DEV(%i) @ BAUD %"PRIu32"\n", dev, baud);

  return 0;
}

static const shell_command_t shell_commands[] = {
  //{ "send", "Send ", cmd_send },
  { NULL, NULL, NULL }
};

int main(void)
{
  uint32_t baud = 115200L;
  unsigned dev = 1;
  uint16_t rate = 10; /* Hz 1-1000 */
  
  printf("Version %s\n", VERSION);
  printf("Number of UART: %i\n", UART_NUMOF);

  if (STDIO_UART_DEV != UART_UNDEF) {
    printf("UART used for STDIO: UART_DEV(%i)\n\n", STDIO_UART_DEV);
  }

  /* initialize ringbuffers */
  for (unsigned i = 0; i < UART_NUMOF; i++) {
    ringbuffer_init(&(ctx[i].rx_buf), ctx[i].rx_mem, UART_BUFSIZE);
  }

  init_serial(dev, baud);

  cmd_reset(dev);
  wait_ms(10);
  cmd_get_fw_version(dev);
  wait_ms(10);
  cmd_set_rate(dev, rate);
  wait_ms(10);
  cmd_restore_factory(dev);
  cmd_save_setting(dev);    

  /* start the parser thread */
  parser_pid = thread_create(parser_stack, sizeof(parser_stack),
			      PARSER_PRIO, 0, parser, NULL, "parser");
  /* run the shell */
  char line_buf[SHELL_BUFSIZE];
  shell_run(shell_commands, line_buf, SHELL_BUFSIZE);
  return 0;
}
