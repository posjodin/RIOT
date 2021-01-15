#define VERSION "0.5 2021-01-15" /* Robert Olsson/KTH 20210-01-13 */
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

#ifndef SHELL_BUFSIZE
#define SHELL_BUFSIZE       (256U)
#endif
#ifndef UART_BUFSIZE
#define UART_BUFSIZE        (256U)
#endif

#define PARSER_PRIO        (THREAD_PRIORITY_MAIN - 1)
#define PARSER_TYPE        (0xabcd)

#ifndef STDIO_UART_DEV
#define STDIO_UART_DEV      (UART_UNDEF)
#endif

#define NOB 134
#define QSIZE 32

static uint8_t YCTa = 0, YCTb = 0;
static uint16_t YCT1=0;
static double dist;
static int thresh = 5;
static kernel_pid_t parser_pid;
static char parser_stack[THREAD_STACKSIZE_MAIN];
static unsigned char buf[NOB];

msg_t msg_queue[QSIZE];
static msg_t msg;

typedef struct {
  char rx_mem[UART_BUFSIZE];
  ringbuffer_t rx_buf;
} uart_ctx_t;

static uart_ctx_t ctx[UART_NUMOF];

struct {
  uint16_t dist, signal;
  int temp;
  time_t time;
} lr;

void wait_ms(int timeout)
{
  uint32_t tmo = timeout * 1000U;
  xtimer_usleep(tmo);
}

inline uint32_t now(void)
{
  return rtt_get_counter();
}

/* From IRQ context */

int sync = 0;

static void rx_cb(void *arg, uint8_t data)
{
  uart_t dev = (uart_t)arg;
  
  if(dev == 1) {
    ringbuffer_add_one(&(ctx[dev].rx_buf), data);
    
    if(data == 0) {
      if(sync == 0) {
	sync = 1;
      }
      if(sync == 1) {
	sync = 2;
      }
      if(sync == 2) {
	sync = 3;
      }
    }
    else {
      sync = 0;
    }
    
    if (sync == 3) {
      msg.content.value = (uint32_t) 1; // MSG_MESSAGE;
      if(msg_try_send(&msg, parser_pid) == 0) {
	puts("Queue full\n");
	msg_init_queue(msg_queue, QSIZE);
      }
      sync = 0;
    }
  }
}

void parse_data(unsigned char *RTT)
{  
    /* Calc obstacle distance of the maximum reflection intensity object */
    YCTa = RTT[3];      
    YCTb = RTT[4];
    YCT1 = (((uint16_t) YCTa) << 8) + YCTb;
    printf("D:  %-5u ", YCT1);

    for(int i = 6; i < NOB-3; i++) {
      if(RTT[i] > thresh) {
	dist = (i-6) * 12.6; /* Calculate distance */
	/* Output the obstacle distance */
	printf("%-3.0f_%-u ", dist, RTT[i]);
      }
    }
    printf("\n");
  //lr.time = now();
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
      //xtimer_usleep(100); // Investigation needed */
    }

    /* Check so reading is ok in beginning and end */

    for(i = 0; i < 3; i++) {
      if(buf[i] != 0xff)
	goto error;
    }
    
    if( buf[NOB-1] || buf[NOB-2] || buf[NOB-3] ) {
	printf("NOT NULL %02X \n", buf[NOB-1]);
	goto error;
    }

#if DEBUG
    for(i = 0; i < NOB; i++) {
      printf(" %02X", buf[i]);
    }
    printf("\n");
#endif
    parse_data(buf);
 error:;
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
  uint32_t baud = 57600L;
  unsigned dev = 1;
  
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
  /* start the parser thread */
  parser_pid = thread_create(parser_stack, sizeof(parser_stack),
			      PARSER_PRIO, 0, parser, NULL, "parser");
  /* run the shell */
  char line_buf[SHELL_BUFSIZE];
  shell_run(shell_commands, line_buf, SHELL_BUFSIZE);
  return 0;
}
