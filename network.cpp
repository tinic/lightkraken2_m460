#include "network.h"

#include "synopGMAC_Dev.h"
#include "nx_m460_eth_driver.h"

#define NX_PACKET_SIZE PKT_FRAME_BUF_SIZE
#define NX_PACKET_POOL_SIZE PKT_FRAME_BUF_SIZE * RECEIVE_DESC_SIZE

Network &Network::instance() {
    static Network network;
    if (!network.initialized) {
        network.initialized = true;
        network.init();
    }
    return network;
}

void Network::init() {
}

static void client_ip_address_changed(NX_IP *ip_ptr, VOID *user) {
    ULONG ip_address = 0;
    ULONG network_mask = 0;
    nx_ip_address_get(ip_ptr, &ip_address, &network_mask);
    printf("client_ip_address_changed %08x %08x\n", ip_address, network_mask);
}

uint8_t *Network::setup(uint8_t *pointer) {
    UINT status = 0;

    const size_t ip_stack_size = 1024;
    const size_t auto_ip_stack_size = 256;
    const size_t arp_cache_size = 1024;

    nx_system_initialize();

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wwrite-strings"

    status = nx_packet_pool_create(&client_pool, "NetX Main Packet Pool", MAX_ETHERNET_PAYLOAD, pointer, NX_PACKET_POOL_SIZE);
    pointer = pointer + NX_PACKET_POOL_SIZE;
    if (status)
        goto fail;

    status = nx_ip_create(&client_ip, "IP", IP_ADDRESS(0, 0, 0, 0), 0xFFFFFF00UL, &client_pool, nx_m460_eth_driver, pointer, ip_stack_size, 1);
    pointer = pointer + ip_stack_size;
    if (status)
        goto fail;

    status = nx_auto_ip_create(&auto_ip, "AutoIP", &client_ip, pointer, auto_ip_stack_size, 1);
    pointer = pointer + auto_ip_stack_size;
    if (status)
        goto fail;

#pragma GCC diagnostic pop

    status = nx_arp_enable(&client_ip, (void *)pointer, arp_cache_size);
    pointer = pointer + arp_cache_size;
    if (status)
        goto fail;

    status = nx_icmp_enable(&client_ip);
    if (status)
        goto fail;

#ifndef BOOTLOADER
    status = nx_udp_enable(&client_ip);
    if (status)
        goto fail;
#endif  // #ifndef BOOTLOADER

    status = nx_tcp_enable(&client_ip);
    if (status)
        goto fail;

#ifndef BOOTLOADER
    status = nx_igmp_enable(&client_ip);
    if (status)
        goto fail;

    status = nxd_ipv6_enable(&client_ip);
    if (status)
        goto fail;
#endif  // #ifndef BOOTLOADER

    status = nx_ip_address_change_notify(&client_ip, client_ip_address_changed, 0);
    if (status)
        goto fail;

    return pointer;

fail:
    while(1) { }
}

static void dhcp_state_change(NX_DHCP *dhcp_ptr, UCHAR new_state)
{
    NX_PARAMETER_NOT_USED(dhcp_ptr);

    switch(new_state) {
        case NX_DHCP_STATE_NOT_STARTED:
        printf("dhcp_state_change NX_DHCP_STATE_NOT_STARTED\n");
        break;
        case NX_DHCP_STATE_BOOT:
        printf("dhcp_state_change NX_DHCP_STATE_BOOT\n");
        break;
        case NX_DHCP_STATE_INIT:
        printf("dhcp_state_change NX_DHCP_STATE_INIT\n");
        break;
        case NX_DHCP_STATE_SELECTING:
        printf("dhcp_state_change NX_DHCP_STATE_SELECTING\n");
        break;
        case NX_DHCP_STATE_REQUESTING:
        printf("dhcp_state_change NX_DHCP_STATE_REQUESTING\n");
        break;
        case NX_DHCP_STATE_BOUND:
        printf("dhcp_state_change NX_DHCP_STATE_BOUND\n");
        break;
        case NX_DHCP_STATE_RENEWING:
        printf("dhcp_state_change NX_DHCP_STATE_RENEWING\n");
        break;
        case NX_DHCP_STATE_REBINDING:
        printf("dhcp_state_change NX_DHCP_STATE_REBINDING\n");
        break;
        case NX_DHCP_STATE_FORCERENEW:
        printf("dhcp_state_change NX_DHCP_STATE_FORCERENEW\n");
        break;
        case NX_DHCP_STATE_ADDRESS_PROBING:
        printf("dhcp_state_change NX_DHCP_STATE_ADDRESS_PROBING\n");
        break;
    }
}

bool Network::start() {

    UINT status = 0;
    ULONG actual_status = 0;

    bool got_ip = false;

    bool try_dhcp = true;
    bool try_autop = true;
    bool try_settings = true;

    /* Wait for the link to come up.  */
    do {
        status =  nx_ip_status_check(&client_ip, NX_IP_LINK_ENABLED, &actual_status, NX_IP_PERIODIC_RATE);
    } while (status != NX_SUCCESS);

    if (!got_ip && try_settings) {
        if (1) {
            nx_ip_address_set(&client_ip, IP_ADDRESS(192, 168, 1, 147), IP_ADDRESS(255, 255, 255, 0));
            got_ip = true;
        }
    }

    if (!got_ip && try_dhcp) {

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wwrite-strings"
        /* Create the DHCP instance.  */
        status = nx_dhcp_create(&dhcp_client, &client_ip, "DHCP Client");
        if (status) {
            return false;
        }
#pragma GCC diagnostic pop

        /* Register state change variable.  */
        status = nx_dhcp_state_change_notify(&dhcp_client, dhcp_state_change);
        if (status) {
            return false;
        }

        /* Start the DHCP Client.  */
        status = nx_dhcp_start(&dhcp_client);
        if (status) {
            return false;
        }

        status = nx_ip_status_check(&client_ip, NX_IP_ADDRESS_RESOLVED, (ULONG *)&actual_status, NX_IP_PERIODIC_RATE * 60);
        if (status == NX_SUCCESS) {
            got_ip = true;
        } else {
            nx_dhcp_stop(&dhcp_client);
            if (status == NX_NOT_SUCCESSFUL) {
                printf("No DHCP address available.\n");
            }
        }

    }

    if (!got_ip && try_autop) {
        status = nx_auto_ip_start(&auto_ip, 0);
        if (status) {
            return false;
        }

        status = nx_ip_status_check(&client_ip, NX_IP_ADDRESS_RESOLVED, (ULONG *)&actual_status, NX_IP_PERIODIC_RATE * 60);
        if (status == NX_SUCCESS) {
            got_ip = true;
        } else {
            nx_auto_ip_stop(&auto_ip);
            if (status == NX_NOT_SUCCESSFUL) {
                printf("No AutoIP address available.\n");
            }
            return false;
        }
    }

    return true;
}
