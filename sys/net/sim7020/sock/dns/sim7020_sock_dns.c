#include <errno.h>
#include "net/ipv4/addr.h"
#include "net/sock/dns.h"
#include "net/sim7020.h"

int sock_dns_query(const char *domain_name, void *addr_out, int family)
{
    char buf[sizeof("255.255.255.255")];
    
    if (family != AF_INET)
        return -EAFNOSUPPORT;
    int res = sim7020_resolve(domain_name, buf);
    if (res < 0)
        return res;
    if (NULL == ipv4_addr_from_str(addr_out, buf))
        return -EINVAL;
    return 0;
}
