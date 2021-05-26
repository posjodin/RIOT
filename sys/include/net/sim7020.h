#ifndef SIM7020_H
#define SIM7020_H

#include "net/netstats.h"

#ifndef SIM7020_UART_DEV
#define SIM7020_UART_DEV UART_DEV(1)
#endif

#ifndef SIM7020_BAUDRATE
#define SIM7020_BAUDRATE 57200
#endif

#ifndef AT_RADIO_MAX_RECV_LEN
#define AT_RADIO_MAX_RECV_LEN 1024
#endif

/* Timeout in seconds to wait for DNS request to complete. Seems to take around 70 seconds to get error code 8 */
#ifndef AT_RADIO_RESOLVE_TIMEOUT
#define AT_RADIO_RESOLVE_TIMEOUT 90
#endif 

#define SIM7020_MAX_SEND_LEN 768

#define SIM7020_MAX_SOCKETS 6
#define SIM7020_NO_SOCKET 7

typedef struct {
    netstats_t ns;
#define tx_unicast_count ns.tx_unicast_count
#define tx_mcast_count ns.tx_mcast_count
#define tx_success ns.tx_success
#define tx_failed ns.tx_failed
#define tx_bytes ns.tx_bytes
#define rx_count ns.rx_count
#define rx_bytes ns.rx_bytes
    uint32_t commfail_count;
    uint32_t reset_count;
    uint32_t activation_count;
    uint32_t activation_fail_count;    
} sim7020_netstats_t;

typedef void (* sim7020_recv_callback_t)(void *, const uint8_t *data, uint16_t datalen);

int sim7020_init(void);
int sim7020_reset(void);
int sim7020_register(void);
int sim7020_activate(void);
int sim7020_status(void);
int sim7020_imsi(char *buf, int len);
int sim7020_imei(char *buf, int len);
int sim7020_udp_socket(const sim7020_recv_callback_t recv_callback, void *recv_callback_arg);
int sim7020_close(uint8_t sockid);
int sim7020_connect(const uint8_t sockid, const sock_udp_ep_t *remote);
int sim7020_bind(const uint8_t sockid, const sock_udp_ep_t *remote);
int sim7020_send(uint8_t sockid, uint8_t *data, size_t datalen);
void *sim7020_recv_thread(void *arg);
int sim7020_resolve(const char *domain, char *result);
sim7020_netstats_t *sim7020_get_netstats(void);
int sim7020_active(void);
int sim7020_at(const char *cmd);
int sim7020_test(uint8_t sockid, int count);

#endif /* SIM7020_H */
