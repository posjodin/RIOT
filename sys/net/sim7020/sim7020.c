/*
 * Copyright (C) 2020 Peter Sjödin, KTH
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>

#include "net/af.h"
#include "net/ipv4/addr.h"
#include "net/ipv6/addr.h"
#include "net/sock/udp.h"

#include "at.h"
#include "xtimer.h"
#include "cond.h"
#include "periph/uart.h"

#include "net/sim7020.h"
#include "net/sim7020_powerkey.h"

#define SIM7020_RECVHEX
#define TCPIPSERIALS

static at_dev_t at_dev;
static char buf[256];
static char resp[1024];

static struct at_radio_status {
    enum {
        AT_RADIO_STATE_NONE,
        AT_RADIO_STATE_IDLE,
        AT_RADIO_STATE_REGISTERED,
        AT_RADIO_STATE_ACTIVE
    } state;
} status = { .state = AT_RADIO_STATE_NONE}; 

struct sock_sim7020 {
    uint8_t sockid;
    sim7020_recv_callback_t recv_callback;
    void *recv_callback_arg;
};
typedef struct sock_sim7020 sim7020_socket_t;

static sim7020_netstats_t netstats;

static sim7020_socket_t sim7020_sockets[SIM7020_MAX_SOCKETS];

mutex_t sim7020_lock = MUTEX_INIT_LOCKED;
#define P(...) 
#define SIM_LOCK() {P("LOCK %d: 0x%x\n", __LINE__, sim7020_lock.queue.next); mutex_lock(&sim7020_lock);}
#define SIM_UNLOCK() {P("UNLOCK %d\n", __LINE__); mutex_unlock(&sim7020_lock);}

/* at_process_urc() allocates line buffer on stack, 
 * so make sure there is room for it
 */
static char sim7020_stack[THREAD_STACKSIZE_DEFAULT + AT_BUF_SIZE];

#define SIM7020_PRIO         (THREAD_PRIORITY_MAIN + 1)
static kernel_pid_t sim7020_pid = KERNEL_PID_UNDEF;

static int _sock_close(uint8_t sockid);
static void *sim7020_thread(void *);

sim7020_netstats_t *sim7020_get_netstats(void) {
    return &netstats;
}

int sim7020_init(void) {

    sim7020_powerkey_init();
    int res = at_dev_init(&at_dev, SIM7020_UART_DEV, SIM7020_BAUDRATE, buf, sizeof(buf));
    if (res != UART_OK) {
        printf("Error initialising AT dev %d speed %d\n", SIM7020_UART_DEV, SIM7020_BAUDRATE);
        return res;
    }
    
    if (pid_is_valid(sim7020_pid)) {
        printf("sim7020 thread already running\n");
    }
    else {
        sim7020_pid = thread_create(sim7020_stack, sizeof(sim7020_stack), SIM7020_PRIO, 0,
                                    sim7020_thread, NULL, "sim7020");
        if (!pid_is_valid(sim7020_pid)) {
            printf("Could not launch sim7020: %d\n", sim7020_pid);
            return sim7020_pid;
        }
    }
    return 0;
}

/* Activation timer to keep track of how long we have been
 * trying to activate the uplink 
 */
static uint8_t _acttimer_expired_flag = 0;
static void _acttimer_cb(void *arg) {
    (void) arg;
    printf("ACTTIMER EXPIRED\n");
    _acttimer_expired_flag = 1;
}

static xtimer_t _acttimer = {.callback = _acttimer_cb, .arg = NULL};

static void _acttimer_start(void) {
    _acttimer_expired_flag = 0;
    printf("ACTTIMER START\n");
    xtimer_set(&_acttimer, 300*1000000);
}

static int _acttimer_expired(void) {
    return _acttimer_expired_flag != 0;
}

/*
 * Reset module by powering it off 
 */
static void _module_reset(void) {
    sim7020_power_off();    
    netstats.reset_count++;
    status.state = AT_RADIO_STATE_NONE;
}

/*
 * Module initialisation -- must be called with module mutex locked 
 */
static int _module_init(void) {
    sim7020_power_on();
    at_dev_poweron(&at_dev);

    int res;
    res = at_send_cmd_wait_ok(&at_dev, "AT+RESET", 5000000);
    /* Ignore */
    res = at_send_cmd_wait_ok(&at_dev, "AT", 5000000);
    if (res < 0) {
        netstats.commfail_count++;
        printf("AT fail\n");
        return res;
    }
    res = at_send_cmd_wait_ok(&at_dev, "AT+CPSMS=0", 5000000);
    if (res < 0) {
        printf("CPSMS fail\n");      
        netstats.commfail_count++;
    }
    /* Limit bands to speed up roaming */
    /* WIP needs a generic solution */
    //res = at_send_cmd_wait_ok(&at_dev, "AT+CBAND=20", 5000000);

#ifdef SIM7020_RECVHEX

    /* Receive data as hex string */
#ifdef TCPIPSERIALS
    /* Show Data in Hex Mode of a Package Received */
    res = at_send_cmd_wait_ok(&at_dev, "AT+CIPHEXS=2", 5000000);
#else
    res = at_send_cmd_wait_ok(&at_dev, "AT+CSORCVFLAG=0", 5000000);
#endif /* TCPIPSERIALS */
#else  
#ifdef TCPIPSERIALS
    /* Show Data in Hex Mode of a Package Received */
    res = at_send_cmd_wait_ok(&at_dev, "AT+CIPHEXS=0", 5000000);
#else
    /* Receive binary data */
    res = at_send_cmd_wait_ok(&at_dev, "AT+CSORCVFLAG=1", 5000000);
#endif /* TCPIPSERIALS */
#endif /* SIM7020_RECVHEX */
    if (res < 0)
        netstats.commfail_count++;
    //Telia is 24001
    //res = at_send_cmd_wait_ok(&at_dev, "AT+COPS=1,2,\"24002\"", 5000000);

    /* Signal Quality Report */
    res = at_send_cmd_get_resp(&at_dev, "AT+CSQ", resp, sizeof(resp), 10*1000000);

    /* Wipe any open sockets */
    uint8_t i;
    for (i = 0; i < SIM7020_MAX_SOCKETS; i++) {
        _sock_close(i);
    }
    res = at_send_cmd_wait_ok(&at_dev, "AT+IPR?", 5000000);
    status.state = AT_RADIO_STATE_IDLE;
    return res;
}

/*
 * Register with operator -- must be called with module mutex locked 
 */
/* Operator MCCMNC (mobile country code and mobile network code) */
/* Telia */
#define OPERATOR "24001"
#define APN "lpwa.telia.iot"
/* Tre  */
//#define OPERATOR "24002"
//#define APN "internet"

int sim7020_register(void) {
    int res;

    /* Force operator selection */
    res =at_send_cmd_wait_ok(&at_dev, "AT+COPS=1,2,\"" OPERATOR "\"", 240*1000000);
    if (res < 0) {
        netstats.commfail_count++;
        return res;
    }

    while (!_acttimer_expired()) {
        res = at_send_cmd_get_resp(&at_dev, "AT+CREG?", resp, sizeof(resp), 120*1000000);
        if (res < 0) {
            netstats.commfail_count++;
        }
        else {
            uint8_t creg;
                
            if (1 == (sscanf(resp, "%*[^:]: %*d,%hhd", &creg))) {
                /* Wait for 1 (Registered, home network) or 5 (Registered, roaming) */
                if (creg == 1 || creg == 5) {
                    status.state = AT_RADIO_STATE_REGISTERED;
                    break;
                }
            }
        }
        xtimer_sleep(5);
    }
    /* Wait for GPRS/Packet Domain attach */
    while (!_acttimer_expired()) {
        res = at_send_cmd_get_resp(&at_dev, "AT+CGATT?", resp, sizeof(resp), 120*1000000);
        if (res > 0) {
            if (0 == (strcmp(resp, "+CGATT: 1")))
                break;
        }
        else
            netstats.commfail_count++;
        xtimer_sleep(5);
    }

    if (_acttimer_expired()) {
        netstats.activation_fail_count++;
        printf("timer expired during activatiion\n");
    }
    return 1;
}

/*
 * Activate data connection -- must be called with module mutex locked 
 */

int sim7020_activate(void) {
    int res;
    uint8_t attempts = 10;
  
    res = at_send_cmd_get_resp(&at_dev,"AT+CIPMUX?", resp, sizeof(resp), 120*1000000);  
    if (res > 0) {
        printf("CIPMUX? resp %s\n", resp);
    }
    else
        printf("CIPMUX? 1 fail %d\n", res); 
    at_drain(&at_dev);

    res = at_send_cmd_get_resp(&at_dev,"AT+CIPMUX=1", resp, sizeof(resp), 120*1000000);  
    if (res < 0) {
        netstats.commfail_count++;
        printf("CIPMUX=1 fail %d\n", res); 
        return res;
    }
    printf("CIPMUX set %s\n", resp);
    at_drain(&at_dev);

    res = at_send_cmd_get_resp(&at_dev,"AT+CIPMUX?", resp, sizeof(resp), 120*1000000);  
    if (res > 0) {
        printf("CIPMUX resp %s\n", resp);
    }
    at_drain(&at_dev);

    /* Already activated? */
    if (strncmp("+CSTT: \"\"", resp, sizeof("+CSTT: \"\"")-1) != 0) {
        status.state = AT_RADIO_STATE_ACTIVE;
    }
    else {
        res = at_send_cmd_get_resp(&at_dev,"AT+CSTT=\"" APN "\",\"\",\"\"", resp, sizeof(resp), 120*1000000);  
        if (res < 0) {
            netstats.commfail_count++;
            return res;
        }
        while (attempts--) {
            /* Bring Up Wireless Connection with GPRS or CSD. This may take a while. */
            printf("Bringing up wireless, be patient\n");
            res = at_send_cmd_wait_ok(&at_dev, "AT+CIICR", 600*1000000);
            if (res == 0) {
                status.state = AT_RADIO_STATE_ACTIVE;
                        break;
            }
            return 0;
        }
    }
    res = at_send_cmd_wait_ok(&at_dev, "AT+CIPSRIP=1", 5000000);
    res = at_send_cmd_wait_ok(&at_dev, "AT+CIPHEAD=1", 5000000);

#ifdef TCPIPSERIALS
    while (attempts--) {
        /* Multi-IP Connection */
        res = at_send_cmd_wait_ok(&at_dev, "AT+CGATT?", 5000000);
        res = at_send_cmd_wait_ok(&at_dev, "AT+CGACT?", 5000000);                
        res = at_send_cmd_wait_ok(&at_dev, "AT+CIPCHAN=0", 5000000);
        res = at_send_cmd_wait_ok(&at_dev, "AT+CIPMUX=1", 5000000);            
        if (res == 0)
            break;
        printf("Get resp %s\n", resp);
        xtimer_sleep(4);
    }
    if (attempts == 0)
        return -1;
#endif    
    return 0;
}

int sim7020_status(void) {
    int res;

    SIM_LOCK();
    if (1) {
        printf("Searching for operators, be patient\n");
        res = at_send_cmd_get_resp(&at_dev, "AT+COPS=?", resp, sizeof(resp), 120*1000000);
    }
    res = at_send_cmd_get_resp(&at_dev, "AT+CREG?", resp, sizeof(resp), 120*1000000);
    /* Request International Mobile Subscriber Identity */
    res = at_send_cmd_get_resp(&at_dev, "AT+CIMI", resp, sizeof(resp), 10*1000000);

    /* Request TA Serial Number Identification (IMEI) */
    res = at_send_cmd_get_resp(&at_dev, "AT+GSN", resp, sizeof(resp), 10*1000000);

    /* Mode 0: Radio information for serving and neighbor cells */
    res = at_send_cmd_wait_ok(&at_dev,"AT+CENG=0", 60*1000000);
    /* Report Network State */
    res = at_send_cmd_get_resp(&at_dev,"AT+CENG?", resp, sizeof(resp), 60*1000000);

    /* Signal Quality Report */
    res = at_send_cmd_wait_ok(&at_dev,"AT+CSQ", 60*1000000);
    /* Task status, APN */
    res = at_send_cmd_get_resp(&at_dev,"AT+CSTT?", resp, sizeof(resp), 60*1000000);

    /* Get Local IP Address */
    res = at_send_cmd_get_resp(&at_dev,"AT+CIFSR", resp, sizeof(resp), 60*1000000);
    /* PDP Context Read Dynamic Parameters */
    res = at_send_cmd_get_resp(&at_dev,"AT+CGCONTRDP", resp, sizeof(resp), 60*1000000);
    SIM_UNLOCK();
    return res;
}


int sim7020_udp_socket(const sim7020_recv_callback_t recv_callback, void *recv_callback_arg) {
    int res;
    /* Create a socket: IPv4, UDP, 1 */

#ifdef TCPIPSERIALS
    {
        static uint8_t sockid = 0;
        sim7020_socket_t *sock = &sim7020_sockets[sockid];
        sock->recv_callback = recv_callback;
        sock->recv_callback_arg = recv_callback_arg;
        return sockid++;
    }
#endif
    SIM_LOCK();
    res = at_send_cmd_get_resp(&at_dev, "AT+CSOC=1,2,1", resp, sizeof(resp), 120*1000000);    
    SIM_UNLOCK();

    if (res > 0) {
        uint8_t sockid;

        if (1 == (sscanf(resp, "+CSOC: %hhd", &sockid))) {
            assert(sockid < SIM7020_MAX_SOCKETS);
            sim7020_socket_t *sock = &sim7020_sockets[sockid];
            sock->recv_callback = recv_callback;
            sock->recv_callback_arg = recv_callback_arg;
            return sockid;
        }
        else {
            if (strcmp("ERROR", resp) == 0)
                return -ENOMEM;
            printf("CSOC parse error: '%s'\n", resp);
        }
    }
    else {
        printf("CSOC failed: %d\n", res);
        netstats.commfail_count++;
        at_drain(&at_dev);
        _module_reset();
    }        
    return res;
}

static int _sock_close(uint8_t sockid) {

    int res;
    char cmd[64];

    assert(sockid < SIM7020_MAX_SOCKETS);
    sim7020_socket_t *sock = &sim7020_sockets[sockid];
    sock->recv_callback = NULL;

    sprintf(cmd, "AT+CSOCL=%d", sockid);
    res = at_send_cmd_wait_ok(&at_dev, cmd, 120*1000000);
    return res;
}

int sim7020_close(uint8_t sockid) {

    SIM_LOCK();
    int res = _sock_close(sockid);
    SIM_UNLOCK();
    return res;
}


#ifdef TCPIPSERIALS
int sim7020_connect(uint8_t sockid, const sock_udp_ep_t *remote) {

    int res;
    char cmd[64];


    assert(sockid < SIM7020_MAX_SOCKETS);

    if (remote->family != AF_INET6) {
        return -EAFNOSUPPORT;
    }
    if (!ipv6_addr_is_ipv4_mapped((ipv6_addr_t *) &remote->addr.ipv6)) {
        printf("sim7020_connect: not ipv6 mapped ipv4: ");
        ipv6_addr_print((ipv6_addr_t *) &remote->addr.ipv6);
        printf("\n");
        return -1;
    }
    if (remote->port == 0) {
        return -EINVAL;
    }

    SIM_LOCK();
    res = at_send_cmd_wait_ok(&at_dev, "AT+CIPMUX=1", 5000000);            
    res = at_send_cmd_wait_ok(&at_dev, "AT+CLPORT= 0, \"UDP\", 8000", 120*1000000);
    //res = at_send_cmd_wait_ok(&at_dev, "AT+CLPORT= \"UDP\", 8000", 120*1000000);
    res = at_send_cmd_wait_ok(&at_dev, "AT+CLPORT?", 120*1000000);
    SIM_UNLOCK();

    char *c = cmd;
    int len = sizeof(cmd);
    int n = snprintf(c, len, "AT+CIPSTART= %u, \"UDP\", ", sockid);
    //int n = snprintf(c, len, "AT+CIPSTART= \"UDP\", ");
    res = at_send_cmd_wait_ok(&at_dev, "AT+CIPMUX=1", 5000000);            
    c += n; len -= n;
    ipv6_addr_t *v6addr = (ipv6_addr_t *) &remote->addr.ipv6;
    ipv4_addr_t *v4addr = (ipv4_addr_t *) &v6addr->u32[3];
    if (NULL == ipv4_addr_to_str(c, v4addr, len)) {
        printf("connect: bad IPv4 mapped address: ");
        ipv6_addr_print((ipv6_addr_t *) &remote->addr.ipv6);
        printf("\n");
        return -1;
    }
    len -= strlen(c);
    c += strlen(c);
    snprintf(c, len, ", %u", remote->port);
    /* Create a socket: IPv4, UDP, 1 */
    SIM_LOCK();
    res = at_send_cmd_wait_ok(&at_dev, cmd, 120*1000000);
    SIM_UNLOCK();
    if (res < 0) {
        netstats.commfail_count++;
        _module_reset();
    }
    SIM_LOCK();
    res = at_send_cmd_wait_ok(&at_dev, "AT+CIPSTATUS=0", 120*1000000);
    SIM_UNLOCK();


    return res;
}

#else
int sim7020_connect(uint8_t sockid, const sock_udp_ep_t *remote) {

    int res;
    char cmd[64];

    assert(sockid < SIM7020_MAX_SOCKETS);

    if (remote->family != AF_INET6) {
        return -EAFNOSUPPORT;
    }
    if (!ipv6_addr_is_ipv4_mapped((ipv6_addr_t *) &remote->addr.ipv6)) {
        printf("sim7020_connect: not ipv6 mapped ipv4: ");
        ipv6_addr_print((ipv6_addr_t *) &remote->addr.ipv6);
        printf("\n");
        return -1;
    }
    if (remote->port == 0) {
        return -EINVAL;
    }

    char *c = cmd;
    int len = sizeof(cmd);
    int n = snprintf(c, len, "AT+CSOCON=%d,%d,", sockid, remote->port);
    c += n; len -= n;
    ipv6_addr_t *v6addr = (ipv6_addr_t *) &remote->addr.ipv6;
    ipv4_addr_t *v4addr = (ipv4_addr_t *) &v6addr->u32[3];
    if (NULL == ipv4_addr_to_str(c, v4addr, len)) {
        printf("connect: bad IPv4 mapped address: ");
        ipv6_addr_print((ipv6_addr_t *) &remote->addr.ipv6);
        printf("\n");
        return -1;
    }

    /* Create a socket: IPv4, UDP, 1 */
    SIM_LOCK();
    res = at_send_cmd_wait_ok(&at_dev, cmd, 120*1000000);
    SIM_UNLOCK();
    if (res < 0) {
        netstats.commfail_count++;
        _module_reset();
    }
    return res;
}
#endif

int sim7020_bind(uint8_t sockid, const sock_udp_ep_t *local) {

    assert(sockid < SIM7020_MAX_SOCKETS);
    if (local->family != AF_INET6) {
        return -EAFNOSUPPORT;
    }

    if (local->port == 0) {
        return -EINVAL;
    }
#if SIM7020_BIND_ENABLE
    printf("binding to to ipv6 mapped ");
    ipv6_addr_print((ipv6_addr_t *) &local->addr.ipv6);
    printf(":%u\n", local->port);

    int res;
    char cmd[64];
    char *c = cmd;
    int len = sizeof(cmd);
    int n = snprintf(c, len, "AT+CSOB=%d,%u", sockid, local->port);

    c += n; len -= n;
    if (!ipv6_addr_is_unspecified((ipv6_addr_t *) &local->addr.ipv6)) {
        ipv6_addr_t *v6addr = (ipv6_addr_t *) &local->addr.ipv6;
        ipv4_addr_t *v4addr = (ipv4_addr_t *) &v6addr->u32[3];
        n = snprintf(c, len, ",");
        c += n; len -= n;
        if (NULL == ipv4_addr_to_str(c, v4addr, len)) {
            printf("bind: bad IPv4 mapped address: ");
            ipv6_addr_print((ipv6_addr_t *) &local->addr.ipv6);
            printf("\n");
            return -1;
        }
    }
    else {
        printf("Bind: unspecified\n");
    }
    SIM_LOCK();

    res = at_send_cmd_wait_ok(&at_dev, cmd, 120*1000000);
    (void) at_send_cmd_wait_ok(&at_dev, "AT+CSOCON?", 120*1000000);    
    res = at_send_cmd_wait_ok(&at_dev, "AT+CSOB?", 120*1000000);

    SIM_UNLOCK();
    printf("socket bound: %d\n", res);
    
    return res;
#else
    (void) sockid;
    return 0;
#endif /* SIM7020_BIND_ENABLE */
}

int sim7020_send(uint8_t sockid, uint8_t *data, size_t datalen) {
    int res;

    char cmd[32];
    size_t len = datalen;

    if (status.state != AT_RADIO_STATE_ACTIVE)
        return -1;

    if (len > SIM7020_MAX_SEND_LEN)
        return -EINVAL;

    assert(sockid < SIM7020_MAX_SOCKETS);
    SIM_LOCK();
    at_drain(&at_dev);
#ifdef TCPIPSERIALS
    snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%u,%u", sockid, len);
    //snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%u", len);
#else
    snprintf(cmd, sizeof(cmd), "AT+CSODSEND=%d,%d", sockid, len);
#endif
    res = at_send_cmd(&at_dev, cmd, 10*1000000);
    res = at_expect_bytes(&at_dev, "> ", 10*1000000);
    if (res != 0) {
        goto fail;
    }
#ifdef TCPIPSERIALS
    res = at_send_cmd_wait_ok(&at_dev, (char *) data, len);
    if (res != 0)
        printf("send afilae\n", res);
    goto out;
#else
    at_send_bytes(&at_dev, (char *) data, len);
    /* Skip empty line */
    (void) at_readline(&at_dev, resp, sizeof(resp), 0, 10*1000000);
    /* Then look for DATA ACCEPT confirmarion */
    res = at_readline(&at_dev, resp, sizeof(resp), 0, 10*1000000);
    if (res < 0) {
        printf("Timeout waiting for DATA ACCEPT confirmation\n");
        goto fail;
    }
    unsigned int nsent;
    if (1 == (sscanf(resp, "DATA ACCEPT: %d", &nsent))) {
        res = nsent;
        netstats.tx_success++;
        netstats.tx_unicast_count++;
        netstats.tx_bytes += datalen;
        goto out;
    }
    printf("No accept: %s\n", resp);
    /* fall through */
#endif
fail:
    netstats.tx_failed++;
    netstats.commfail_count++;
    _module_reset();
out:
    SIM_UNLOCK();
    return res;

}

static mutex_t resolve_mutex;
static cond_t resolve_cond;
static mutex_t resolve_cond_mutex;
static enum {
    R_WAIT, R_DONE, R_TIMEOUT, R_ERROR
} resolve_state;

/*
 * URC callback for a response like: +CDNSGIP: 1,"lab-pc.ssvl.kth.se","192.16.125.232" 
 * Argument arg is pointer to char buffer where lookup result should be stored. 
 */
static void _resolve_urc_cb(void *arg, const char *code) {
    char *result = arg;
    const char *resp = code;

    mutex_lock(&resolve_cond_mutex);
    if (1 == sscanf(resp, "+CDNSGIP: %*d,\"%*[^\"]\",\"%[^\"]", result)) {
        resolve_state = R_DONE;
    }
    else {
        resolve_state = R_ERROR;
    }
    cond_signal(&resolve_cond);
    mutex_unlock(&resolve_cond_mutex);
}

static void _resolve_timeout_cb(void *arg)
{
    (void) arg;
    mutex_lock(&resolve_cond_mutex);
    printf("resolve timeout\n");
    resolve_state = R_TIMEOUT;
    cond_signal(&resolve_cond);
    mutex_unlock(&resolve_cond_mutex);
}

int sim7020_resolve(const char *domain, char *result) {
    static at_urc_t urc;
    xtimer_t timeout_timer;
    int res;
    
    /* Only one at a time */
    mutex_lock(&resolve_mutex); 
    resolve_state = R_WAIT;
    urc.cb = _resolve_urc_cb;
    urc.code = "+CDNSGIP:";
    urc.arg = result;
    at_add_urc(&at_dev, &urc);
    
    timeout_timer.callback = _resolve_timeout_cb;
    xtimer_set(&timeout_timer, 10*1000000U);

    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+CDNSGIP=%s", domain);
    SIM_LOCK();
    res = at_send_cmd_wait_ok(&at_dev, cmd, 120*1000000);
    SIM_UNLOCK();
    if (res < 0)
        goto out;

    mutex_lock(&resolve_cond_mutex);
    while (resolve_state == R_WAIT)
        cond_wait(&resolve_cond, &resolve_cond_mutex);
    if (resolve_state != R_TIMEOUT)
        xtimer_remove(&timeout_timer);

    if (resolve_state != R_DONE)
        res = -1;
    else 
        res = 0;

out:
    mutex_unlock(&resolve_cond_mutex);
    at_remove_urc(&at_dev, &urc);
    mutex_unlock(&resolve_mutex);
    return res;
}

static uint8_t recv_buf[AT_RADIO_MAX_RECV_LEN];
#ifdef TCPIPSERIALS
static void _receive_cb(void *arg, const char *code) {
    (void) arg;
    int sockid, len;

    int res = sscanf(code, "+RECEIVE,%d,%d:", &sockid, &len);
    printf("RECEIVE CB %d:  '%s'\n", res, code);
    if (res == 2) {

#ifdef SIM7020_RECVHEX
        res = at_readline(&at_dev, (char *) recv_buf, sizeof(recv_buf), 0, 10*1000000);
        printf("Read %d byte '%s'\n", res, recv_buf);
        /* Data is encoded as hex string, so
         * data length is half the string length */ 
        int rcvlen = len >> 1;
        if (rcvlen >  AT_RADIO_MAX_RECV_LEN)
            return; /* Too large */

        /* Find first char after second comma */
        char *ptr = strchr(strchr(code, ',')+1, ',')+1;

        /* Copy into receive buffer */
        for (int i = 0; i < rcvlen; i++) {
            char hexstr[3];
            hexstr[0] = *ptr++; hexstr[1] = *ptr++; hexstr[2] = '\0';
            recv_buf[i] = (uint8_t) strtoul(hexstr, NULL, 16);
        }
#else
        /* Data is binary */
        int rcvlen = len;
        if (rcvlen >  AT_RADIO_MAX_RECV_LEN)
            return; /* Too large */

        /* Find first char after second comma */
        char *ptr = strchr(strchr(code, ',')+1, ',')+1;

        /* Copy into receive buffer */
        memcpy(recv_buf, ptr, rcvlen);
#endif /* SIM7020_RECVHEX */

#if 0
        for (int i = 0; i < rcvlen; i++) {
            if (isprint(recv_buf[i]))
                putchar(recv_buf[i]);
            else
                printf("x%02x", recv_buf[i]);
            putchar(' ');
        }
        putchar('\n');
#endif
        if (sockid >= SIM7020_MAX_SOCKETS) {
            printf("Illegal sim socket %d\n", sockid);
            return;
        }
        sim7020_socket_t *sock = &sim7020_sockets[sockid];
        if (sock->recv_callback != NULL) {
            sock->recv_callback(sock->recv_callback_arg, recv_buf, rcvlen);
        }
        else {
            printf("sockid %d: no callback\n", sockid);
        }
        netstats.rx_count++;
        netstats.rx_bytes += rcvlen;
    }
    else
        printf("csonmi_cb res %d\n", res);
}
#else
static void _csonmi_cb(void *arg, const char *code) {
    (void) arg;
    int sockid, len;

    int res = sscanf(code, "+CSONMI: %d,%d,", &sockid, &len);
    if (res == 2) {

#ifdef SIM7020_RECVHEX

        /* Data is encoded as hex string, so
         * data length is half the string length */ 
        int rcvlen = len >> 1;
        if (rcvlen >  AT_RADIO_MAX_RECV_LEN)
            return; /* Too large */

        /* Find first char after second comma */
        char *ptr = strchr(strchr(code, ',')+1, ',')+1;

        /* Copy into receive buffer */
        for (int i = 0; i < rcvlen; i++) {
            char hexstr[3];
            hexstr[0] = *ptr++; hexstr[1] = *ptr++; hexstr[2] = '\0';
            recv_buf[i] = (uint8_t) strtoul(hexstr, NULL, 16);
        }
#else
        /* Data is binary */
        int rcvlen = len;
        if (rcvlen >  AT_RADIO_MAX_RECV_LEN)
            return; /* Too large */

        /* Find first char after second comma */
        char *ptr = strchr(strchr(code, ',')+1, ',')+1;

        /* Copy into receive buffer */
        memcpy(recv_buf, ptr, rcvlen);
#endif /* SIM7020_RECVHEX */

#if 0
        for (int i = 0; i < rcvlen; i++) {
            if (isprint(recv_buf[i]))
                putchar(recv_buf[i]);
            else
                printf("x%02x", recv_buf[i]);
            putchar(' ');
        }
        putchar('\n');
#endif
        if (sockid >= SIM7020_MAX_SOCKETS) {
            printf("Illegal sim socket %d\n", sockid);
            return;
        }
        sim7020_socket_t *sock = &sim7020_sockets[sockid];
        if (sock->recv_callback != NULL) {
            sock->recv_callback(sock->recv_callback_arg, recv_buf, rcvlen);
        }
        else {
            printf("sockid %d: no callback\n", sockid);
        }
        netstats.rx_count++;
        netstats.rx_bytes += rcvlen;
    }
    else
        printf("csonmi_cb res %d\n", res);
}

#endif /* TCPIPSERIALS */

#define URC_POLL_MSECS 1000
static void _recv_loop(void) {
    at_urc_t urc;

#ifdef TCPIPSERIALS
    urc.cb = _receive_cb;
    urc.code = "+RECEIVE,";
#else
    urc.cb = _csonmi_cb;
    urc.code = "+CSONMI:";
#endif 
    urc.arg = NULL;
    at_add_urc(&at_dev, &urc);
    while (status.state == AT_RADIO_STATE_ACTIVE) {
        static int c;
        if ((c++ % 100) == 0)
            printf("recv loop\n");
        SIM_LOCK();
        at_process_urc(&at_dev, URC_POLL_MSECS*(uint32_t) 0100);
        SIM_UNLOCK();
    }
    at_remove_urc(&at_dev, &urc);
}


static void *sim7020_thread(void *arg) {
    (void) arg;

again:
    printf("again sim socket: locked 0x%x\n", sim7020_lock.queue.next);
    switch (status.state) {
    case AT_RADIO_STATE_NONE:
        printf("***module init:\n");
        _acttimer_start();
        _module_init();
        goto again;
    case AT_RADIO_STATE_IDLE:
        printf("***register:\n");
        sim7020_register();
         goto again;
    case AT_RADIO_STATE_REGISTERED:
        printf("***activate:\n");
        sim7020_activate();
        if (status.state == AT_RADIO_STATE_ACTIVE)
            SIM_UNLOCK();
        goto again;
    case AT_RADIO_STATE_ACTIVE:
        printf("***recv loop:\n");
        _recv_loop();
        goto again;
    }
    return NULL;
}

int sim7020_active(void) {
    return status.state == AT_RADIO_STATE_ACTIVE;
}

int sim7020_test(uint8_t sockid, int count) {
    (void) sockid;
    (void) count;
    char buf[64];
    int res = sim7020_resolve("lab-pc.ssvl.kth.se", buf);
    
    if (res == 0)
        printf("Resolve: %s\n", buf);
    return 0;
}
