#include "tx_api.h"
#include "nx_api.h"
#include "fx_api.h"
#include "nx_auto_ip.h"
#include "nxd_dhcp_client.h"
#include "nxd_http_server.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wregister"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wpedantic"

#include "NuMicro.h"
#include "nx_m460_eth_driver.h"
#include "synopGMAC_Dev.h"

#pragma GCC diagnostic pop

#include "fs.h"

#define NX_PACKET_SIZE PKT_FRAME_BUF_SIZE
#define NX_PACKET_POOL_SIZE PKT_FRAME_BUF_SIZE * TRANSMIT_DESC_SIZE

#define LED_INIT() (PH->MODE = ((PH->MODE & (~(0x3ful << 4 * 2))) | (0x15ul << 4 * 2)))
#define LED_RED PH4
#define LED_YELLOW PH5
#define LED_GREEN PH6

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wwrite-strings"

static NX_IP client_ip {};
static NX_AUTO_IP auto_ip {};
static NX_DHCP dhcp_client {};
static NX_HTTP_SERVER http_server {};
static NX_PACKET_POOL client_pool {};
static FX_MEDIA ram_disk {};
static unsigned char media_memory[512];

static void dhcp_state_change(NX_DHCP *dhcp_ptr, UCHAR new_state)
{
    NX_PARAMETER_NOT_USED(dhcp_ptr);
    NX_PARAMETER_NOT_USED(new_state);

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

extern "C" void _fx_ram_driver(FX_MEDIA *media_ptr);

static TX_THREAD thread_startup {};
void thread_startup_entry(ULONG thread_input)
{
    NX_PARAMETER_NOT_USED(thread_input);

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
            goto start_web_server;
        }
    }

    if (!got_ip && try_dhcp) {
        /* Create the DHCP instance.  */
        status = nx_dhcp_create(&dhcp_client, &client_ip, "DHCP Client");
        if (status) {
            return;
        }

        /* Register state change variable.  */
        status = nx_dhcp_state_change_notify(&dhcp_client, dhcp_state_change);
        if (status) {
            return;
        }

        /* Start the DHCP Client.  */
        status = nx_dhcp_start(&dhcp_client);
        if (status) {
            return;
        }

        status = nx_ip_status_check(&client_ip, NX_IP_ADDRESS_RESOLVED, (ULONG *)&actual_status, NX_IP_PERIODIC_RATE * 60);
        if (status == NX_SUCCESS) {
            got_ip = true;
            goto start_web_server;
        } else {
            if (status == NX_NOT_SUCCESSFUL) {
                printf("No DHCP address available.\n");
            }
        }

        nx_dhcp_stop(&dhcp_client);
    }

    if (!got_ip && try_autop) {
        status = nx_auto_ip_start(&auto_ip, 0);
        if (status) {
            return;
        }

        status = nx_ip_status_check(&client_ip, NX_IP_ADDRESS_RESOLVED, (ULONG *)&actual_status, NX_IP_PERIODIC_RATE * 60);
        if (status == NX_SUCCESS) {
            got_ip = true;
            goto start_web_server;
        } else {
            if (status == NX_NOT_SUCCESSFUL) {
                printf("No AutoIP address available.\n");
            }
        }

        nx_auto_ip_stop(&auto_ip);
    }

start_web_server:

    if (!got_ip) {
        // Should never happen!
        return;
    }

    status =  fx_media_open(&ram_disk, "RAM Disk", _fx_ram_driver, fs_data, media_memory, sizeof(media_memory));
    if (status) {
        return;
    }

    // Now we can start the web server
    status = nx_http_server_start(&http_server);
    if (status) {
        return;
    }
}

static void client_ip_address_changed(NX_IP *ip_ptr, VOID *user) {
    ULONG ip_address = 0;
    ULONG network_mask = 0;
    nx_ip_address_get(ip_ptr, &ip_address, &network_mask);
    printf("client_ip_address_changed %08x %08x\n", ip_address, network_mask);
}

extern "C" void tx_application_define(void *first_unused_memory);

void tx_application_define(void *first_unused_memory)
{
    UINT status = 0;
    uint8_t *pointer = (uint8_t *)first_unused_memory;

    fx_system_initialize();

    const size_t startup_stack_size = 1024;
    tx_thread_create(&thread_startup, "startup", thread_startup_entry, 0, pointer, startup_stack_size, 1, 1, TX_NO_TIME_SLICE, TX_AUTO_START);
    pointer = pointer + startup_stack_size;

    nx_system_initialize();

    status = nx_packet_pool_create(&client_pool, "NetX Main Packet Pool", MAX_ETHERNET_PAYLOAD, pointer, NX_PACKET_POOL_SIZE);
    pointer = pointer + NX_PACKET_POOL_SIZE;
    if (status)
        return;

    const size_t ip_stack_size = 1024;
    status = nx_ip_create(&client_ip, "IP", IP_ADDRESS(0, 0, 0, 0), 0xFFFFFF00UL, &client_pool, nx_m460_eth_driver, pointer, ip_stack_size, 1);
    pointer = pointer + ip_stack_size;
    if (status)
        return;

    const size_t auto_ip_stack_size = 256;
    status = nx_auto_ip_create(&auto_ip, "AutoIP", &client_ip, pointer, auto_ip_stack_size, 1);
    pointer = pointer + auto_ip_stack_size;
    if (status)
        return;

    const size_t arp_cache_size = 1024;
    status = nx_arp_enable(&client_ip, (void *)pointer, arp_cache_size);
    pointer = pointer + arp_cache_size;
    if (status)
        return;

    status = nx_icmp_enable(&client_ip);
    if (status)
        return;

    status = nx_udp_enable(&client_ip);
    if (status)
        return;

    status = nx_tcp_enable(&client_ip);
    if (status)
        return;

    status = nx_igmp_enable(&client_ip);
    if (status)
        return;

    status = nxd_ipv6_enable(&client_ip);
    if (status)
        return;

    status = nx_ip_address_change_notify(&client_ip, client_ip_address_changed, 0);
    if (status)
        return;

    const size_t http_server_stack_size = 2048;
    status = nx_http_server_create(&http_server, "HTTP Server", &client_ip, &ram_disk, pointer, http_server_stack_size, &client_pool, NX_NULL, NX_NULL);
    pointer = pointer + http_server_stack_size;
    if (status) {
        return;
    }

    static NX_HTTP_SERVER_MIME_MAP map[] = {
        {"js",     "text/javascript"},
        {"css",    "text/css"},
        {"json",   "application/json"},
        {"svg",    "image/svg+xml"}
    };

    nx_http_server_mime_maps_additional_set(&http_server, map, 4);

    printf("Consumed %d bytes of RAM.\n", (int)(pointer-(uint8_t *)first_unused_memory));
}
