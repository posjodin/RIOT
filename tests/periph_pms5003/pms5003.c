/*
 * Copyright (c) 2017, Peter Sjodin, KTH Royal Institute of Technology
 * All rights reserved.
 *
 * Author  : Peter Sjodin, KTH Royal Institute of Technology
 * Created : 2017-04-21
 *
 * Hacked by Robert Olsson 2021-01-19
 */

/*
 * \file
 *         Driver for Planttower PMSX003 dust sensors
 */

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

uint16_t i2c_addr = PMS5003_I2C_ADDR;
i2c_t i2c_dev = 0;

#define PM_TREAD_PRIO        (THREAD_PRIORITY_MAIN - 1)
#define PM_TREAD_TYPE        (0xabcd)

static kernel_pid_t pms5003_thread_pid;
static char pms5003_thread_stack[THREAD_STACKSIZE_MAIN];

uint8_t buf[PMSBUFFER];

/* Frame assembly statistics */
static uint32_t invalid_frames, valid_frames;

/* Sensor configured on? */
uint8_t configured_on = 0;

/* Last readings of sensor data */
static uint16_t PM1, PM2_5, PM10;
static uint16_t PM1_ATM, PM2_5_ATM, PM10_ATM;
static uint16_t DB0_3, DB0_5, DB1, DB2_5, DB5, DB10;
/* Time when last sensor data was read, in clock_seconds()*/
//static unsigned long timestamp = 0;

struct pms_config {
  unsigned sample_period;    /* Time between samples (sec) */
  unsigned warmup_interval; /* Warmup time (sec) */
} pms_config;

uint32_t now(void)
{
  return rtt_get_counter();
}

uint16_t pms5003_pm1(void)
{
  return PM1;
}
uint16_t pms5003_pm2_5(void)
{
  return PM2_5;
}
uint16_t pms5003_pm10(void)
{
  return PM10;
}
uint16_t pms5003_pm1_atm(void)
{
  return PM1_ATM;
}
uint16_t pms5003_pm2_5_atm(void)
{
  return PM2_5_ATM;
}
uint16_t pms5003_pm10_atm(void)
{
  return PM10_ATM;
}
uint16_t pms5003_db0_3(void)
{
  return DB0_3;
}
uint16_t pms5003_db0_5(void)
{
  return DB0_5;
}
uint16_t pms5003_db1(void)
{
  return DB1;
}
uint16_t pms5003_db2_5(void)
{
  return DB2_5;
}
uint16_t pms5003_db5(void)
{
  return DB5;
}
uint16_t pms5003_db10(void)
{
  return DB10;
}
uint32_t pms5003_timestamp(void)
{
  return now();
}
uint32_t pms5003_valid_frames(void)
{
  return valid_frames;
}
uint32_t pms5003_invalid_frames(void)
{
  return invalid_frames;
}

void pms5003_config_sample_period(unsigned int sample_period) {
  pms_config.sample_period = sample_period;
}

void pms5003_config_warmup_interval(unsigned int warmup_interval) {
  pms_config.warmup_interval = warmup_interval;
}

unsigned pms5003_get_sample_period(void) {
  return pms_config.sample_period;
}

unsigned pms5003_get_warmup_interval(void) {
  return pms_config.warmup_interval;
}

/**
 * Sensor API for PMS5003
 */

/**
 * Validate frame by checking preamble, length field and checksum.
 * Return 0 if invalid frame, otherwise 1.
 */
static int check_pmsframe(uint8_t *buf)
{
  int sum, pmssum;
  int i;
  int len;

  if(buf[0] != PRE1 || buf[1] != PRE2) {
    return 0;
  }
  /* len is length of frame not including preamble and checksum */
  len = (buf[2] << 8) + buf[3];
  if(len < PMSMINBODYLEN || len > PMSMAXBODYLEN) {
    return 0;
  }
  /* Sum data bytewise, including preamble but excluding checksum */
  sum = 0;
  for(i = 0; i < len + 2; i++) {
    sum += buf[i];
  }
  /* Compare with received checksum last in frame*/
  pmssum = (buf[len + 2] << 8) + buf[len + 3];
  return pmssum == sum;
}

void printpm(void)
{
  printf("PMS frames: valid %lu, invalid %lu\n",
         valid_frames, invalid_frames);
  printf("PM1=%-u PM2.5=%-u PM10=%-u\n", PM1, PM2_5, PM10);
  printf("PM1_ATM=%-u PM2.5_ATM=%-u PM10_ATM=%-u\n",
         PM1_ATM, PM2_5_ATM, PM10_ATM);
  printf(" DB0_3=%-u DB0_5=%-u DB1=%-u DB2_5=%-u DB5=%-u DB10=%-u\n",
         DB0_3, DB0_5, DB1, DB2_5, DB5, DB10);
}

/**
 * Frame received from PMS sensor. Validate and update sensor data.
 * Return 1 if valid frame, otherwise 0
 */
int pmsframe(uint8_t *buf)
{
  int len;
  
  if(! check_pmsframe(buf)) {
    invalid_frames++;
    return 0;
  }
  //timestamp = now();
  valid_frames++;
  /* Update sensor readings */
  PM1 = (buf[4] << 8) | buf[5];
  PM2_5 = (buf[6] << 8) | buf[7];
  PM10 = (buf[8] << 8) | buf[9];
  PM1_ATM = (buf[10] << 8) | buf[11];
  PM2_5_ATM = (buf[12] << 8) | buf[13];
  PM10_ATM = (buf[14] << 8) | buf[15];
  /* Not all Plantower sensors report dust size bins. 
   * PMS3003 (frame length 20) doesn't. 
   * PMS5003 (frame length 28) does.
   * Use length field to detect if the frame has size bins. 
   */
  len = (buf[2] << 8) + buf[3];
  if(len == 28) {
    DB0_3 = (buf[16] << 8) | buf[17];
    DB0_5 = (buf[18] << 8) | buf[19];
    DB1 = (buf[20] << 8) | buf[21];
    DB2_5 = (buf[22] << 8) | buf[23];
    DB5 = (buf[24] << 8) | buf[25];
    DB10 = (buf[26] << 8) | buf[27];
  }
  return 1;
}

/**
 * Initialize. Create event, and start timer-driven process.
 * If UART enabled, also install UART callback function and
 * start PMS frame assembly process.
 */

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

void wait_ms(uint32_t timeout)
{
  uint32_t tmo = timeout * 1000U;
  xtimer_usleep(tmo);
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
#ifdef DEBUG
  printpm();
#endif
  return res;
}

static void *pms5003_thread(void *arg)
{
  (void)arg;

  uint32_t interval = pms_config.sample_period * 1000;

  while (1) {
    wait_ms(interval);
    read_pms5002(i2c_dev, i2c_addr);
  }
  return NULL;
}

void pms5003_init(void)
{
  i2c_probe(i2c_dev, i2c_addr);

  pms5003_config_sample_period(PMS_SAMPLE_PERIOD);
  pms5003_config_warmup_interval(PMS_WARMUP_INTERVAL);
  configured_on = 1;

  /* start the parser thread */
  pms5003_thread_pid = thread_create(pms5003_thread_stack, sizeof(pms5003_thread_stack),
			      PM_TREAD_PRIO, 0, pms5003_thread, NULL, "pm_tread");
  
#ifdef DEBUG
  printf("PMS5003: sample period %d, warmup interval %d\n",
         pms_config.sample_period, pms_config.warmup_interval);
#endif /* DEBUG */
}
