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
#include "fs.h"

#include "tx_api.h"
#include "nx_api.h"
#include "nxd_dhcp_client.h"
#include "nx_auto_ip.h"

#include "nx_m460_eth_driver.h"
#include "synopGMAC_network_interface.h"

#define NX_PACKET_SIZE PKT_FRAME_BUF_SIZE
#define NX_PACKET_POOL_SIZE PKT_FRAME_BUF_SIZE * TRANSMIT_DESC_SIZE

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

static TX_THREAD thread_blink = {0};
void thread_blink_entry(ULONG thread_input)
{
//    int32_t i = 0;

    /* Enable GPIO PH to control LED */
    CLK->AHBCLK0 |= CLK_AHBCLK0_GPHCKEN_Msk;

    LED_INIT();
    LED_YELLOW = 1;
    LED_RED = 0;
    LED_GREEN = 0;

    volatile int32_t d = 0;
    for (size_t c = 0; c < sizeof(fs_data); c++) {
        d += fs_data[c];
    }

    while (1) {
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
    }
}

static NX_IP client_ip = { 0 };
static NX_AUTO_IP auto_ip = { 0 };
static NX_DHCP dhcp_client = { 0 };

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

static TX_THREAD thread_resolveip = { 0 };
void thread_resolveip_entry(ULONG thread_input)
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
        if (0) {
            nx_ip_address_set(&client_ip, IP_ADDRESS(192, 168, 1, 143), IP_ADDRESS(255, 255, 255, 0));
            got_ip = true;
            return;
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
            return;
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
            return;
        } else {
            if (status == NX_NOT_SUCCESSFUL) {
                printf("No AutoIP address available.\n");
            }
        }

        nx_auto_ip_stop(&auto_ip);
    }

    if (!got_ip) {
        // Should never happen!
    }
}

static void client_ip_address_changed(NX_IP *ip_ptr, VOID *user) {
    ULONG ip_address = 0;
    ULONG network_mask = 0;
    nx_ip_address_get(ip_ptr, &ip_address, &network_mask);
    printf("client_ip_address_changed %08x %08x\n", ip_address, network_mask);
}

void tx_application_define(void *first_unused_memory)
{
    UINT status = 0;
    uint8_t *pointer = (uint8_t *)first_unused_memory;

    static NX_PACKET_POOL client_pool = { 0 };

    const size_t blink_stack_size = 256;
    tx_thread_create(&thread_blink, "blink", thread_blink_entry, 0, pointer, blink_stack_size, 1, 1, TX_NO_TIME_SLICE, TX_AUTO_START);
    pointer = pointer + blink_stack_size;

    const size_t resolveip_stack_size = 1024;
    tx_thread_create(&thread_resolveip, "resolveip", thread_resolveip_entry, 0, pointer, resolveip_stack_size, 1, 1, TX_NO_TIME_SLICE, TX_AUTO_START);
    pointer = pointer + resolveip_stack_size;

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

    status = nx_ip_address_change_notify(&client_ip, client_ip_address_changed, 0);
    if (status)
        return;

    printf("Consumed %d bytes of RAM.\n", (int)(pointer-(uint8_t *)first_unused_memory));
}

int main()
{
    SYS_Lightkraken_Init();

    tx_kernel_enter();
}
