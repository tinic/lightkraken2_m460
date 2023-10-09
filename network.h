#ifndef _NETWORK_H_
#define _NETWORK_H_

#include <stdint.h>

#include "tx_api.h"
#include "nx_api.h"
#include "nx_auto_ip.h"
#include "nxd_dhcp_client.h"

class Network {
public:
    static Network &instance();

    uint8_t *setup(uint8_t *pointer);
    bool start();

    NX_IP *ip() { return &client_ip; };
    NX_PACKET_POOL *pool() { return &client_pool; }

private:

    void init();
    bool initialized = false;

    NX_IP client_ip {};
    NX_AUTO_IP auto_ip {};
    NX_DHCP dhcp_client {};
    NX_PACKET_POOL client_pool {};
};

#endif  // #ifndef _NETWORK_H_
