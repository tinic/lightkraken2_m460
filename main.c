/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    A simple demo for NuMaker-M467HJ board to show message from UART0 to ICE VCOM and show LED
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "clk.h"
#include "uart.h"
#include "gpio.h"
#include "sys.h"
// #include "fs.h"

#include "tx_api.h"
#include "nx_api.h"
#include "nxd_dhcp_client.h"

#include "nx_m460_eth_driver.h"
#include "synopGMAC_network_interface.h"

#define NX_PACKET_SIZE PKT_FRAME_BUF_SIZE
#define NX_PACKET_POOL_SIZE PKT_FRAME_BUF_SIZE * 8

#define LED_INIT() (PH->MODE = ((PH->MODE & (~(0x3ful << 4 * 2))) | (0x15ul << 4 * 2)))
#define LED_RED PH4
#define LED_YELLOW PH5
#define LED_GREEN PH6

void SYS_Init(void);
void UART0_Init(void);

void SYS_Lightkraken_Init()
{

    // Enable ICE
    SYS->GPF_MFP0 &= ~(SYS_GPF_MFP0_PF1MFP_Msk | SYS_GPF_MFP0_PF0MFP_Msk);
    SYS->GPF_MFP0 |= (SYS_GPF_MFP0_PF1MFP_ICE_CLK | SYS_GPF_MFP0_PF0MFP_ICE_DAT);

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* If the macros do not exist in your project, please refer to the related clk.h in Header folder of the tool package */
    /* Enable clock source */
    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk | CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for clock source ready */
    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk | CLK_STATUS_HXTSTB_Msk);

    /* Disable PLL first to avoid unstable when setting PLL */
    CLK_DisablePLL();

    /* Set PLL frequency */
    CLK->PLLCTL = (CLK->PLLCTL & ~(0x008FFFFFUL)) | 0x00004212UL;

    /* Waiting for PLL ready */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Disable PLLFN first to avoid unstable when setting PLLFN */
    CLK->PLLCTL |= CLK_PLLFNCTL1_PD_Msk;

    /* Set PLLFN frequency */
    CLK->PLLFNCTL0 = (CLK->PLLFNCTL0 & ~(0x0FFFFFFFUL)) | 0x00000417UL;

    /* Disable PLLFN first to avoid unstable when setting PLLFN */
    CLK->PLLCTL |= CLK_PLLFNCTL1_PD_Msk;

    /* Set PLLFN frequency */
    CLK->PLLFNCTL1 = (CLK->PLLFNCTL1 & ~(0xF8000000UL)) | 0x00000000UL;

    /* Set HCLK clock */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(1));

    /* Set PCLK-related clock */
    CLK->PCLKDIV = (CLK_PCLKDIV_PCLK0DIV2 | CLK_PCLKDIV_PCLK1DIV2);

    /* Enable IP clock */
    CLK_EnableModuleClock(EMAC0_MODULE);
    CLK_EnableModuleClock(FMCIDLE_MODULE);
    CLK_EnableModuleClock(GPA_MODULE);
    CLK_EnableModuleClock(GPB_MODULE);
    CLK_EnableModuleClock(GPC_MODULE);
    CLK_EnableModuleClock(GPD_MODULE);
    CLK_EnableModuleClock(GPE_MODULE);
    CLK_EnableModuleClock(PDMA0_MODULE);
    CLK_EnableModuleClock(PDMA1_MODULE);
    CLK_EnableModuleClock(RTC_MODULE);
    CLK_EnableModuleClock(SPI0_MODULE);
    CLK_EnableModuleClock(SPI1_MODULE);
    CLK_EnableModuleClock(SPI2_MODULE);
    CLK_EnableModuleClock(SPI3_MODULE);
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(TMR1_MODULE);
    CLK_EnableModuleClock(TMR2_MODULE);
    CLK_EnableModuleClock(TMR3_MODULE);
    CLK_EnableModuleClock(TRNG_MODULE);
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(UART1_MODULE);
    CLK_EnableModuleClock(UART2_MODULE);
    CLK_EnableModuleClock(UART3_MODULE);
    CLK_EnableModuleClock(WDT_MODULE);
    CLK_EnableModuleClock(WWDT_MODULE);

    CLK_EnableCKO(CLK_CLKSEL1_CLKOSEL_PLLFN_DIV2, 1, 0);
    CLK->CLKOCTL = CLK->CLKOCTL & ~CLK_CLKOCTL_CLK1HZEN_Msk;
    CLK_EnableSysTick(CLK_CLKSEL0_STCLKSEL_HCLK_DIV2, 0);

    /* Set IP clock */
    CLK_SetModuleClock(RTC_MODULE, RTC_LXTCTL_RTCCKSEL_LIRC, MODULE_NoMsk);
    //    CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL2_SPI0SEL_PCLK1, CLK_SPI0_CLKDIV_SPI0(1));
    //    CLK_SetModuleClock(SPI1_MODULE, CLK_CLKSEL2_SPI1SEL_PCLK0, CLK_SPI1_CLKDIV_SPI1(1));
    //    CLK_SetModuleClock(SPI2_MODULE, CLK_CLKSEL3_SPI2SEL_PCLK1, CLK_SPI2_CLKDIV_SPI2(1));
    //    CLK_SetModuleClock(SPI3_MODULE, CLK_CLKSEL3_SPI3SEL_PCLK0, CLK_SPI3_CLKDIV_SPI3(1));
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK0, MODULE_NoMsk);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_PCLK0, MODULE_NoMsk);
    CLK_SetModuleClock(TMR2_MODULE, CLK_CLKSEL1_TMR2SEL_PCLK1, MODULE_NoMsk);
    CLK_SetModuleClock(TMR3_MODULE, CLK_CLKSEL1_TMR3SEL_PCLK1, MODULE_NoMsk);
    CLK_SetModuleClock(TRNG_MODULE, CLK_CLKSEL2_TRNGSEL_LIRC, MODULE_NoMsk);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_HXT, CLK_CLKDIV0_UART1(1));
    CLK_SetModuleClock(UART2_MODULE, CLK_CLKSEL3_UART2SEL_HXT, CLK_CLKDIV4_UART2(1));
    CLK_SetModuleClock(UART3_MODULE, CLK_CLKSEL3_UART3SEL_HXT, CLK_CLKDIV4_UART3(1));
    CLK_SetModuleClock(WDT_MODULE, CLK_CLKSEL1_WDTSEL_LIRC, MODULE_NoMsk);
    CLK_SetModuleClock(WWDT_MODULE, CLK_CLKSEL1_WWDTSEL_HCLK_DIV2048, MODULE_NoMsk);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    // CLKO -> 25Mhz, PB14 JP7.33
    SET_CLKO_PB14();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

    UART0->LINE = UART_PARITY_NONE | UART_STOP_BIT_1 | UART_WORD_LEN_8;
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HXT, 115200);

    /* Lock protected registers */
    SYS_LockReg();
}

static TX_THREAD thread_0 = {0};

void thread_0_entry(ULONG thread_input)
{
    int32_t i = 0;

    /* Enable GPIO PH to control LED */
    CLK->AHBCLK0 |= CLK_AHBCLK0_GPHCKEN_Msk;

    printf("\n");
    printf("+------------------------------------------------------------------+\n");
    printf("|              Simple Blinky Demo                                  |\n");
    printf("+------------------------------------------------------------------+\n");

    LED_INIT();
    LED_YELLOW = 1;
    LED_RED = 0;
    LED_GREEN = 0;

    while (1)
    {

        LED_YELLOW = 0;
        LED_RED = 1;
        LED_GREEN = 1;
        tx_thread_sleep(20);
        LED_YELLOW = 1;
        LED_RED = 0;
        LED_GREEN = 1;
        tx_thread_sleep(20);
        LED_YELLOW = 1;
        LED_RED = 1;
        LED_GREEN = 0;
        tx_thread_sleep(20);

        // printf("Iter %d\n", i++);
    }
}

static TX_THREAD thread_1 = {0};
static NX_IP client_ip = {0};
static NX_DHCP dhcp_client = {0};
static ULONG state_changes = 0;
static ULONG error_counter = 0;
static ULONG client_thread_counter = 0;
static UCHAR message[50] = "My Ping Request!";
static NX_PACKET_POOL client_pool = {0};

void _nx_ram_network_driver(NX_IP_DRIVER *driver_req_ptr);

#define NX_DHCP_INTERFACE_INDEX 0

static void dhcp_state_change(NX_DHCP *dhcp_ptr, UCHAR new_state)
{
    NX_PARAMETER_NOT_USED(dhcp_ptr);
    NX_PARAMETER_NOT_USED(new_state);

    /* Increment state changes counter.  */
    state_changes++;

    return;
}

void thread_1_entry(ULONG thread_input)
{
    UINT status;
    UINT actual_status;
    UINT length;
    UINT ping = NX_TRUE;
    UINT run_dhcp_client = NX_TRUE;
    NX_PACKET *my_packet;

    NX_PARAMETER_NOT_USED(thread_input);

    /* Modified the mtu size to avoid fragmenting the DHCP packet since the default mtu size is 128 in _nx_ram_network_driver.  */
    status = nx_ip_interface_mtu_set(&client_ip, NX_DHCP_INTERFACE_INDEX, 1500);

    /* Check for MTU set errors.  */
    if (status)
        return;

    /* Create the DHCP instance.  */
    status = nx_dhcp_create(&dhcp_client, &client_ip, "DHCP-CLIENT");
    if (status)
        return;

#ifdef REQUEST_CLIENT_IP
    /* Request a specific IP address using the DHCP client address option. */
    status = nx_dhcp_request_client_ip(&dhcp_client, NX_DHCP_CLIENT_IP_ADDRESS, SKIP_DISCOVER_MESSAGE);
    if (status)
        error_counter++;
#endif

    /* Register state change variable.  */
    status = nx_dhcp_state_change_notify(&dhcp_client, dhcp_state_change);
    if (status)
        error_counter++;

    /* Start the DHCP Client.  */
    nx_dhcp_start(&dhcp_client);
    while (run_dhcp_client)
    {
        /* Wait for DHCP to assign the IP address.  */
        do
        {

            /* Check for address resolution.  */
            status = nx_ip_status_check(&client_ip, NX_IP_ADDRESS_RESOLVED, (ULONG *)&actual_status, NX_IP_PERIODIC_RATE);

            /* Check status.  */
            if (status)
            {
                /* wait a bit. */
                tx_thread_sleep(NX_IP_PERIODIC_RATE);
            }

        } while (status != NX_SUCCESS);

        length = sizeof(message);

        while (ping)
        {
            /* Send pings to another host on the network...  */
            status = nx_icmp_ping(&client_ip, IP_ADDRESS(192, 168, 0, 1), (CHAR *)message, length, &my_packet, NX_IP_PERIODIC_RATE);
            if (status)
                error_counter++;
            else
                nx_packet_release(my_packet);

            /* Increment counter.  */
            client_thread_counter++;

            /* Sleep for a few ticks...  */
            tx_thread_sleep(NX_IP_PERIODIC_RATE);
        }

        /* Use this API to send a message to the server, e.g. a DECLINE if the IP address is owned by another host.
        nx_dhcp_send_request(&dhcp_client, NX_DHCP_TYPE_DHCPDECLINE);
        */

        /* Use this API to release an IP address if the host is switching networks or running the host through DHCP cycles.
        nx_dhcp_release(&dhcp_client);
        */

        /* Stopping the DHCP client. */
        nx_dhcp_stop(&dhcp_client);

        tx_thread_sleep(NX_IP_PERIODIC_RATE);

        /* Use this API to clear the network parameters and restart the client in the INIT state. */
        nx_dhcp_reinitialize(&dhcp_client);

        /* Resume the DHCP client thread. */
        nx_dhcp_start(&dhcp_client);

        /* Ok to resume ping attempts. */
        ping = NX_TRUE;
    }

    /* All done. Return resources to NetX and ThreadX. */
    nx_dhcp_delete(&dhcp_client);

    return;
}

void tx_application_define(void *first_unused_memory)
{
    UINT status = 0;
    void *pointer = first_unused_memory;

    tx_thread_create(&thread_0, "thread 0", thread_0_entry, 0, pointer, 1024, 1, 1, TX_NO_TIME_SLICE, TX_AUTO_START);
    pointer = pointer + 1024;

    tx_thread_create(&thread_0, "thread 1", thread_1_entry, 0, pointer, 4096, 1, 1, TX_NO_TIME_SLICE, TX_AUTO_START);
    pointer = pointer + 4096;

    /* Initialize the NetX system.  */
    nx_system_initialize();

    /* Create the client packet pool.  */
    status = nx_packet_pool_create(&client_pool, "NetX Main Packet Pool", 1024, pointer, NX_PACKET_POOL_SIZE);
    pointer = pointer + NX_PACKET_POOL_SIZE;

    /* Check for pool creation error.  */
    if (status)
        return;

    /* Create an IP instance for the DHCP Client.  */
    status = nx_ip_create(&client_ip, "DHCP Client", IP_ADDRESS(0, 0, 0, 0), 0xFFFFFF00UL, &client_pool, _nx_ram_network_driver, pointer, 2048, 1);
    pointer = pointer + 2048;

    /* Check for IP create errors.  */
    if (status)
        return;

    /* Enable ARP and supply ARP cache memory for DHCP Client IP.  */
    status = nx_arp_enable(&client_ip, (void *)pointer, 1024);
    pointer = pointer + 1024;

    /* Check for ARP enable errors.  */
    if (status)
        return;

    /* Enable UDP traffic.  */
    status = nx_udp_enable(&client_ip);

    /* Check for UDP enable errors.  */
    if (status)
        return;

    /* Enable ICMP.  */
    status = nx_icmp_enable(&client_ip);

    /* Check for errors.  */
    if (status)
        return;
}

int main()
{
    SYS_Lightkraken_Init();

    tx_kernel_enter();
}
