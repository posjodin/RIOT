#define VERSION "0.9 2021-01-13"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "board.h"
#include "shell.h"
#include "thread.h"
#include "msg.h"
#include "ringbuffer.h"
#include "periph/uart.h"
#include "xtimer.h"

#ifdef MODULE_STDIO_UART
#include "stdio_uart.h"
#endif

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

static uart_ctx_t ctx[UART_NUMOF];
static kernel_pid_t parser_pid;
static char parser_stack[THREAD_STACKSIZE_MAIN];
unsigned char buf[120];

void wait_ms(int timeout)
{
  uint32_t tmo = timeout * 1000U;
  xtimer_usleep(tmo);
}

int lidar_version(int dev)
{
  /* Version 5A 04 01 5F */
  
  if(1)  {
    buf[0] = 0x5A;
    buf[1] = 0x04;
    buf[2] = 0x01;
    buf[3] = 0x5F;
    uart_write(UART_DEV(dev), buf, 4);
  }
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
  uint16_t dist, signal;
  int temp;

  //printf("CS frame-cs=%02x csum=%02x\n", buf[8], csum(buf, 8));
  if(buf[8] != csum(buf, 8)) {
    return; // csum err
  }
  
  dist = buf[2] + buf[3]*256;
  signal = buf[4] + buf[5]*256;
  temp = buf[6] + buf[7]*256;
  temp = temp/8 -256;
  printf("Dist=%-5u Signal=%-5u Temp=%-d", dist, signal, temp);
  printf("\r");
  
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
  printf("parse_command\n");

  for(i=0; i < 7; i++)
    printf(" %02x", buf[i]);
  printf("\n");
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
    while  (! ringbuffer_empty(&(ctx[dev].rx_buf))) {
      buf[i++] = (int)ringbuffer_get_one(&(ctx[dev].rx_buf));
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

static int init(int dev, uint32_t baud)
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

  printf("Version %s\n", VERSION);
  
  printf("Available devices:               %i\n", UART_NUMOF);

  if (STDIO_UART_DEV != UART_UNDEF) {
    printf("UART used for STDIO (the shell): UART_DEV(%i)\n\n", STDIO_UART_DEV);
  }

  /* initialize ringbuffers */
  for (unsigned i = 0; i < UART_NUMOF; i++) {
    ringbuffer_init(&(ctx[i].rx_buf), ctx[i].rx_mem, UART_BUFSIZE);
  }

  /* start the parser thread */
  parser_pid = thread_create(parser_stack, sizeof(parser_stack),
			      PARSER_PRIO, 0, parser, NULL, "parser");
  init(dev, baud);
  lidar_version(dev);
    
  /* run the shell */
  char line_buf[SHELL_BUFSIZE];
  shell_run(shell_commands, line_buf, SHELL_BUFSIZE);
  return 0;
}
