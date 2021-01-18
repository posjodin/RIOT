/*
 * Copyright (c) 2017, Peter Sjodin, KTH Royal Institute of Technology
 * All rights reserved.
 *
 * Author  : Peter Sjodin, KTH Royal Institute of Technology
 * Created : 2017-04-21
 *
 * Hacked by Robert Olsson
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

/* Two preamble bytes */
#define PRE1 0x42
#define PRE2 0x4d
/* Valid values for body length field */
#define PMSMINBODYLEN 20
#define PMSMAXBODYLEN 28
/* Buffer holds frame body plus preamble (two bytes)
 * and length field (two bytes) */
#define PMSBUFFER (PMSMAXBODYLEN + 4)
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

void printpm_old(void)
{
  printf("PMS frames: valid %lu, invalid %lu\n",
         valid_frames, invalid_frames);
  printf("PM1=%-u PM2.5=%-u PM10=%-u\n", PM1, PM2_5, PM10);
  printf("PM1_ATM=%-u PM2.5_ATM=%-u PM10_ATM=%-u\n",
         PM1_ATM, PM2_5_ATM, PM10_ATM);
  printf(" DB0_3=%-u DB0_5=%-u DB1=%-u DB2_5=%-u DB5=%-u DB10=%-u\n",
         DB0_3, DB0_5, DB1, DB2_5, DB5, DB10);
}

void printpm(void)
{
  printf("%-u %-u %-u", PM1, PM2_5, PM10);
  printf("  %-u %-u %-u",
         PM1_ATM, PM2_5_ATM, PM10_ATM);
  printf("   %-u %-u %-u %-u %-u %-u\n",
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
