/*
 * Copyright (C) 2020 Robert Olsson
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     examples
 * @{
 *
 * @file
 * @brief       WSSN application
 *
 * @author      Robert Olsson <roolss@kth.se>
 *
 * @}
 */

#include <stdio.h>
#include <string.h>
#include "thread.h"
#include "xtimer.h"
#include "timex.h"
#include "shell.h"

#ifdef MODULE_NETIF
#include "net/gnrc/pktdump.h"
#include "net/gnrc.h"
#endif

#ifndef SHELL_BUFSIZE
#define SHELL_BUFSIZE       (128U)
#endif

#define SENSD_TAG   "&:  "

/* set interval to 10 second */
#define INTERVAL (30U * US_PER_SEC)
#define DEF_TTL 0xF

uint8_t addr[2];
uint8_t l2addr[2];
uint8_t lqi;
int16_t rssi;
#define R_TXT        ((uint64_t) ((uint64_t) 1)<<0)  
#define R_EPC        ((uint64_t) ((uint64_t) 1)<<1)

char *epc_str= "34323001D900000000000001";

gnrc_netif_t *iface = NULL;

#define MAXTXTSIZE 10
//unsigned char txt [MAXTXTSIZE];
char *txt = "RIOT";
int8_t seqno;
#define MAX_BCAST_SIZE 99

int tx_pkt(gnrc_netif_t *iface);

struct payload {
  uint16_t channel;
  unsigned char dummy[2];
  uint8_t head; /* version << 4 + ttl */
  uint8_t seqno;
  uint8_t buf[MAX_BCAST_SIZE+20];  /* Check for max payload 20 extra to be able to test */
};

struct payload p;
extern kernel_pid_t rime_pid;

struct {
  uint16_t sent;
  uint16_t dup;
  uint16_t ignored;
  uint16_t ttl_zero;
} relay_stats;

#define MAX_NEIGHBORS 64
#define NEIGHBOR_TIMEOUT 200 * CLOCK_SECOND /* 64000 */

struct neighbor {
  struct neighbor *next;
  //linkaddr_t addr;
  uint8_t last_seqno;
  /* The ->last_rssi and ->last_lqi fields hold the Received Signal
     Strength Indicator (RSSI) and CC2420 Link Quality Indicator (LQI)
     values that are received for the incoming broadcast packets. */

  uint8_t last_rssi;
  uint8_t last_lqi;
  uint8_t last_ttl;

  /* Best RSSI and the TTL for this transmisson
     This in case we use relaying
   */
  uint8_t best_rssi;
  uint8_t best_ttl;

  uint8_t dup;
  /* The ->avg_gap contains the average seqno gap that we have seen
     from this neighbor. */
  uint32_t avg_seqno_gap;
  //struct ctimer ctimer;
};

static int cmd_tx(int argc, char **argv)
{

  if (argc > 2) {
    printf("%s: just tx \n", argv[0]);
    return 1;
  }
  
  tx_pkt(iface);
  return 0;
}

static const shell_command_t shell_commands[] = {
  { "tx", "tx RIME pkt", cmd_tx },
  { NULL, NULL, NULL }
};

int addr_cmp(uint8_t *a, uint8_t *b)
{

  if((a[0] == b[0]) && (a[1] == b[1]))
    return 1;
  else
    return 0;
}

int relay = 0;

int rx_pkt(gnrc_pktsnip_t *pkt)
{
  struct payload *pl;

  if( pkt->type == GNRC_NETTYPE_NETIF) {
    gnrc_netif_hdr_t *hdr = pkt->data;

    if (hdr->src_l2addr_len == 2) {
      uint8_t *laddr;

      laddr = gnrc_netif_hdr_get_src_addr(hdr);
      addr[0] = *laddr;
      laddr++;
      addr[1] = *laddr;
    }
    if (hdr->src_l2addr_len == 8) {
      uint8_t *laddr;

      laddr = gnrc_netif_hdr_get_src_addr(hdr);
      addr[0] = *laddr;
      laddr++;
      addr[1] = *laddr;
    }
    lqi = hdr->lqi;
    rssi = hdr->rssi;
    return 0;
  }

  /* Parse payload */
  
  if( pkt->type != GNRC_NETTYPE_UNDEF)
    return -1;
  
  pl =(struct payload *) pkt->data;
 
  if((pl->head >> 4) != 1) {
    relay = 0;
    relay_stats.ignored++;
    return -1;
  }

  if((pl->head & 0xF) == 0) {
    relay = 0;
    relay_stats.ttl_zero++;
    return -1;
  }

  /* From our own address. Can happen if we receive own pkt via relay
     Ignore
  */

  if(addr_cmp(l2addr, addr)) {
    relay = 0;
    relay_stats.ignored++;
    printf("SAME\n");
    //return;
  }

  printf("&: %s ", &pl->buf);
  printf(" [ADDR=%-d.%-d SEQ=%-d TTL=%-u RSSI=%-d LQI=%-u]\n", addr[0], addr[1], pl->seqno,
	 (pl->head & 0x0F), (signed) rssi, (unsigned) lqi);
  
#if 0
      printf(" [ADDR=%-d.%-d SEQ=%-d TTL=%-u RSSI=%-u LQI=%-u DRP=%-d.%02d DELAY=%d]",
	     p->addr[0], p->addr[1], p->seqno, p->ttl, p->rssi, p->lqi,  p->drp[0], p->drp[1], 
	     (uptime - p->time));
#endif
      return 0;
}

void rime_msg(gnrc_pktsnip_t *pkt)
{
  gnrc_pktsnip_t *snip = pkt;

  pkt = gnrc_pktsnip_search_type(snip, GNRC_NETTYPE_NETIF);
  if(pkt)
    rx_pkt(pkt);

  pkt = gnrc_pktsnip_search_type(snip, GNRC_NETTYPE_UNDEF);
  if(pkt)
    rx_pkt(pkt);
  
  gnrc_pktbuf_release(snip);
}

int len_exceeded(int len)
{
  if(len > MAX_BCAST_SIZE) {
      printf("Error TX Length=%d\n", len);
    return 1;
  }
  return 0;
}

int tx_pkt(gnrc_netif_t *iface)
{
  gnrc_pktsnip_t *pkt;
  struct payload pl;
  uint16_t len = 0;
  uint64_t mask;
  uint8_t ttl = DEF_TTL;
  gnrc_netif_hdr_t *hdr;
  gnrc_pktsnip_t *netif;
  size_t payload_len;
  
  mask = (R_TXT|R_EPC); /* For test */

  if(mask & R_TXT) {
    len += snprintf((char *) &pl.buf[len], sizeof(pl.buf), 
		    "TXT=%s ", txt);
    
    if( len_exceeded(len)) 
      return -1;
  }
  if(mask & R_EPC) {
    len += snprintf((char *) &pl.buf[len], sizeof(pl.buf), 
		    "EPC=%s ", epc_str);
    
    if( len_exceeded(len)) 
      return -1;
  }

  pl.channel = 0x81; /* 129 for Contiki */
  pl.dummy[0] = 0xAB;
  pl.dummy[1] = 0xBA;
  pl.head = (1<<4); /* Version 1 for sensd */
  pl.head |= ttl;
  pl.seqno = seqno++;
  pl.buf[len++] = 0;

  payload_len = strlen((char *) pl.buf)+7;

  pkt = gnrc_pktbuf_add(NULL, &pl, payload_len, GNRC_NETTYPE_UNDEF);
    
  if(!pkt) 
    return -1;
    
  netif = gnrc_netif_hdr_build(NULL, 0, NULL, 0);
    
  if(!netif)
    return -1;
    
  hdr = netif->data;
  hdr->flags |= GNRC_NETIF_HDR_FLAGS_BROADCAST;
  LL_PREPEND(pkt, netif);
  gnrc_netif_send(iface, pkt);
  return 1;
}

int main(void)
{
  int res;
  uint16_t src_len;
  gnrc_netif_t *netif = NULL;
  uint16_t pan = 0xabcd;
  uint16_t chan = 26;
  //xtimer_ticks32_t last_wakeup = xtimer_now();
 
#ifdef MODULE_NETIF
  gnrc_netreg_entry_t dump = GNRC_NETREG_ENTRY_INIT_PID(GNRC_NETREG_DEMUX_CTX_ALL, rime_pid);
  gnrc_netreg_register(GNRC_NETTYPE_UNDEF, &dump);
#endif
  
  gnrc_pktdump_init();
  
  /* Get the 802154 interface */
  while ((netif = gnrc_netif_iter(netif))) {
    iface = gnrc_netif_get_by_pid(netif->pid);
    printf("Interface: pid=%d device_type=%d\n", iface->pid, iface->device_type);

    if(iface->device_type == NETDEV_TYPE_IEEE802154) {
      break;
    }
  }

  if(!iface)
    return -1;
  
  src_len = 8U; /* To get the correct addr */
  res = gnrc_netapi_set(iface->pid, NETOPT_SRC_LEN,0, &src_len, sizeof(src_len));

  if(res < 0 )
    printf("ERR set src_len\n");
  
  res = gnrc_netapi_get(iface->pid, NETOPT_ADDRESS,0, &l2addr, sizeof(src_len));
  
  if(res < 0 )
    printf("ERR get addr\n");

  /* Compat w. Contiki/sensd */
  l2addr[0] = iface->l2addr[7];
  l2addr[1] = iface->l2addr[6];
  printf("Main Short_addr: %-d.%-d\n", l2addr[0], l2addr[1]);
  
  src_len = 2U; /* To get the correct addr */
  res = gnrc_netapi_set(iface->pid, NETOPT_SRC_LEN,0, &src_len, sizeof(src_len));
  
  if(res < 0 )
   printf("ERR set src_len\n");
  
  res = gnrc_netapi_set(iface->pid, NETOPT_ADDRESS,0, &l2addr, sizeof(src_len));
 
  if(res < 0 )
    printf("ERR set addr\n");
 
  res = netif_set_opt(&iface->netif, NETOPT_NID, 0, &pan, sizeof(pan));
  if(res < 0)
    printf("Setting PAN failed\n");
  
  res = netif_get_opt(&iface->netif, NETOPT_CHANNEL, 0, &chan, sizeof(chan));
  if (res < 0)
    printf("Setting Chan failed\n");
  
  char line_buf[SHELL_BUFSIZE];
  shell_run(shell_commands, line_buf, SHELL_BUFSIZE);

}
