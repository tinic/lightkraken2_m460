/**************************************************************************/
/*                                                                        */
/*       Copyright (c) Microsoft Corporation. All rights reserved.        */
/*                                                                        */
/*       This software is licensed under the Microsoft Software License   */
/*       Terms for Microsoft Azure RTOS. Full text of the license can be  */
/*       found in the LICENSE file at https://aka.ms/AzureRTOS_EULA       */
/*       and in the root directory of this software.                      */
/*                                                                        */
/**************************************************************************/

/* Indicate that driver source is being compiled.  */

#define NX_DRIVER_SOURCE

/****** DRIVER SPECIFIC ****** Start of part/vendor specific include area.  Include driver-specific include file here!  */

#ifndef NX_M460_ETH_DRIVER_H

/* Determine if the driver uses IP deferred processing or direct ISR processing.  */

#define NX_DRIVER_ENABLE_DEFERRED                /* Define this to enable deferred ISR processing.  */

/* Include driver specific include file.  */
#include "NuMicro.h"
#include "nx_m460_eth_driver.h"

#endif /* NX_M60_ETH_DRIVER_H */

/****** DRIVER SPECIFIC ****** End of part/vendor specific include file area!  */

/* Define the driver information structure that is only available within this file.  */
/* Place Ethernet BD at uncacheable memory*/
static NX_DRIVER_INFORMATION nx_driver_information;

/* Rounded header size */
static ULONG header_size;

/****** DRIVER SPECIFIC ****** Start of part/vendor specific data area.  Include hardware-specific data here!  */

static synopGMACdevice GMACdev = {0};
static bool synopGMACdeviceInit = false;
static DmaDesc tx_desc[TRANSMIT_DESC_SIZE] __attribute__((aligned(32))) = {0};
static DmaDesc rx_desc[RECEIVE_DESC_SIZE] __attribute__((aligned(32))) = {0};
static PKT_FRAME_T tx_buf[TRANSMIT_DESC_SIZE] __attribute__((aligned(32))) = {0};
static PKT_FRAME_T rx_buf[RECEIVE_DESC_SIZE] __attribute__((aligned(32))) = {0};

/****** DRIVER SPECIFIC ****** End of part/vendor specific data area!  */

/* Define the routines for processing each driver entry request.  The contents of these routines will change with
each driver. However, the main driver entry function will not change, except for the entry function name.  */

static VOID         _nx_driver_interface_attach(NX_IP_DRIVER *driver_req_ptr);
static VOID         _nx_driver_initialize(NX_IP_DRIVER *driver_req_ptr);
static VOID         _nx_driver_enable(NX_IP_DRIVER *driver_req_ptr);
static VOID         _nx_driver_disable(NX_IP_DRIVER *driver_req_ptr);
static VOID         _nx_driver_packet_send(NX_IP_DRIVER *driver_req_ptr);
static VOID         _nx_driver_multicast_join(NX_IP_DRIVER *driver_req_ptr);
static VOID         _nx_driver_multicast_leave(NX_IP_DRIVER *driver_req_ptr);
static VOID         _nx_driver_get_status(NX_IP_DRIVER *driver_req_ptr);
#ifdef NX_ENABLE_INTERFACE_CAPABILITY
static VOID         _nx_driver_capability_get(NX_IP_DRIVER *driver_req_ptr);
static VOID         _nx_driver_capability_set(NX_IP_DRIVER *driver_req_ptr);
#endif /* NX_ENABLE_INTERFACE_CAPABILITY */
#ifdef NX_DRIVER_ENABLE_DEFERRED
static VOID         _nx_driver_deferred_processing(NX_IP_DRIVER *driver_req_ptr);
#endif  // #ifdef NX_DRIVER_ENABLE_DEFERRED
static VOID         _nx_driver_transfer_to_netx(NX_IP *ip_ptr, NX_PACKET *packet_ptr);


/* Define the prototypes for the hardware implementation of this driver. The contents of these routines are
driver-specific.  */

static UINT         _nx_driver_hardware_initialize(NX_IP_DRIVER *driver_req_ptr);
static UINT         _nx_driver_hardware_enable(NX_IP_DRIVER *driver_req_ptr);
static UINT         _nx_driver_hardware_disable(NX_IP_DRIVER *driver_req_ptr);
static UINT         _nx_driver_hardware_packet_send(NX_PACKET *packet_ptr);
static UINT         _nx_driver_hardware_multicast_join(NX_IP_DRIVER *driver_req_ptr);
static UINT         _nx_driver_hardware_multicast_leave(NX_IP_DRIVER *driver_req_ptr);
static UINT         _nx_driver_hardware_get_status(NX_IP_DRIVER *driver_req_ptr);
static bool         _nx_driver_hardware_packet_received();
#ifdef NX_ENABLE_INTERFACE_CAPABILITY
static UINT         _nx_driver_hardware_capability_set(NX_IP_DRIVER *driver_req_ptr);
#endif /* NX_ENABLE_INTERFACE_CAPABILITY */

static int32_t      _nx_mii_mdio_read(synopGMACdevice *gmacdev, uint16_t reg, uint16_t *val);
static int32_t      _nx_mii_mdio_write(synopGMACdevice *gmacdev, uint16_t reg, uint16_t val);
static int32_t      _nx_mii_link_ok(synopGMACdevice *gmacdev);
static int32_t      _nx_mii_check_phy_init(synopGMACdevice *gmacdev);

//static void         _nx_hex_dump(char *desc, void *addr, int len);

/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    nx_m460_eth_driver                                                 */
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Yuxin Zhou, Microsoft Corporation                                   */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This is the entry point of the NetX Ethernet Driver. This driver    */
/*    function is responsible for initializing the Ethernet controller,   */
/*    enabling or disabling the controller as need, preparing             */
/*    a packet for transmission, and getting status information.          */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    driver_req_ptr                        The driver request from the   */
/*                                            IP layer.                   */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _nx_driver_interface_attach           Process attach request        */
/*    _nx_driver_initialize                 Process initialize request    */
/*    _nx_driver_enable                     Process link enable request   */
/*    _nx_driver_disable                    Process link disable request  */
/*    _nx_driver_packet_send                Process send packet requests  */
/*    _nx_driver_multicast_join             Process multicast join request*/
/*    _nx_driver_multicast_leave            Process multicast leave req   */
/*    _nx_driver_get_status                 Process get status request    */
/*    _nx_driver_deferred_processing        Drive deferred processing     */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    IP layer                                                            */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Yuxin Zhou               Initial Version 6.0           */
/*  xx-xx-xxxx     Yuxin Zhou               Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/
/****** DRIVER SPECIFIC ****** Start of part/vendor specific global driver entry function name.  */
VOID  nx_m460_eth_driver(NX_IP_DRIVER *driver_req_ptr)
/****** DRIVER SPECIFIC ****** End of part/vendor specific global driver entry function name.  */
{

  /* Default to successful return.  */
  driver_req_ptr -> nx_ip_driver_status =  NX_SUCCESS;

  /* Process according to the driver request type in the IP control
  block.  */
  switch (driver_req_ptr -> nx_ip_driver_command)
  {

  case NX_LINK_INTERFACE_ATTACH:

    /* Process link interface attach requests.  */
    _nx_driver_interface_attach(driver_req_ptr);
    break;

  case NX_LINK_INITIALIZE:
    {

      /* Process link initialize requests.  */
      _nx_driver_initialize(driver_req_ptr);
      break;
    }

  case NX_LINK_ENABLE:
    {

      /* Process link enable requests.  */
      _nx_driver_enable(driver_req_ptr);
      break;
    }

  case NX_LINK_DISABLE:
    {

      /* Process link disable requests.  */
      _nx_driver_disable(driver_req_ptr);
      break;
    }


  case NX_LINK_ARP_SEND:
  case NX_LINK_ARP_RESPONSE_SEND:
  case NX_LINK_PACKET_BROADCAST:
  case NX_LINK_RARP_SEND:
  case NX_LINK_PACKET_SEND:
    {

      /* Process packet send requests.  */
      _nx_driver_packet_send(driver_req_ptr);
      break;
    }


  case NX_LINK_MULTICAST_JOIN:
    {

      /* Process multicast join requests.  */
      _nx_driver_multicast_join(driver_req_ptr);
      break;
    }


  case NX_LINK_MULTICAST_LEAVE:
    {

      /* Process multicast leave requests.  */
      _nx_driver_multicast_leave(driver_req_ptr);
      break;
    }

  case NX_LINK_GET_STATUS:
    {

      /* Process get status requests.  */
      _nx_driver_get_status(driver_req_ptr);
      break;
    }

#ifdef NX_DRIVER_ENABLE_DEFERRED
  case NX_LINK_DEFERRED_PROCESSING:
    {

      /* Process driver deferred requests.  */

      /* Process a device driver function on behave of the IP thread. */
      _nx_driver_deferred_processing(driver_req_ptr);

      break;
    }
#endif // #ifdef NX_DRIVER_ENABLE_DEFERRED


#ifdef NX_ENABLE_INTERFACE_CAPABILITY
  case NX_INTERFACE_CAPABILITY_GET:
    {

      /* Process get capability requests.  */
      _nx_driver_capability_get(driver_req_ptr);
      break;
    }

  case NX_INTERFACE_CAPABILITY_SET:
    {

      /* Process set capability requests.  */
      _nx_driver_capability_set(driver_req_ptr);
      break;
    }
#endif /* NX_ENABLE_INTERFACE_CAPABILITY */

  default:


    /* Invalid driver request.  */

    /* Return the unhandled command status.  */
    driver_req_ptr -> nx_ip_driver_status =  NX_UNHANDLED_COMMAND;

    /* Default to successful return.  */
    driver_req_ptr -> nx_ip_driver_status =  NX_DRIVER_ERROR;
  }
}


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _nx_driver_interface_attach                                         */
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Yuxin Zhou, Microsoft Corporation                                   */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function processing the interface attach request.              */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    driver_req_ptr                        Driver command from the IP    */
/*                                            thread                      */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Driver entry function                                               */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Yuxin Zhou               Initial Version 6.0           */
/*  xx-xx-xxxx     Yuxin Zhou               Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/
static VOID  _nx_driver_interface_attach(NX_IP_DRIVER *driver_req_ptr)
{


  /* Setup the driver's interface.  This example is for a simple one-interface
  Ethernet driver. Additional logic is necessary for multiple port devices.  */
  nx_driver_information.nx_driver_information_interface =  driver_req_ptr -> nx_ip_driver_interface;

#ifdef NX_ENABLE_INTERFACE_CAPABILITY
  driver_req_ptr -> nx_ip_driver_interface -> nx_interface_capability_flag = NX_DRIVER_CAPABILITY;
#endif /* NX_ENABLE_INTERFACE_CAPABILITY */

  /* Return successful status.  */
  driver_req_ptr -> nx_ip_driver_status =  NX_SUCCESS;
}


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _nx_driver_initialize                                               */
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Yuxin Zhou, Microsoft Corporation                                   */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function processing the initialize request.  The processing    */
/*    in this function is generic. All ethernet controller logic is to    */
/*    be placed in _nx_driver_hardware_initialize.                        */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    driver_req_ptr                        Driver command from the IP    */
/*                                            thread                      */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _nx_driver_hardware_initialize        Process initialize request    */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Driver entry function                                               */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Yuxin Zhou               Initial Version 6.0           */
/*  xx-xx-xxxx     Yuxin Zhou               Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/
static VOID  _nx_driver_initialize(NX_IP_DRIVER *driver_req_ptr)
{

  NX_IP           *ip_ptr;
  NX_INTERFACE    *interface_ptr;
  UINT            status;
  CHAR           *payload_address;       /* Address of the first payload*/
  VOID           *rounded_pool_start;    /* Rounded stating address     */

  /* Setup the IP pointer from the driver request.  */
  ip_ptr =  driver_req_ptr -> nx_ip_driver_ptr;

  /* Setup interface pointer.  */
  interface_ptr = driver_req_ptr -> nx_ip_driver_interface;

  /* Initialize the driver's information structure.  */

  /* Default IP pointer to NULL.  */
  nx_driver_information.nx_driver_information_ip_ptr =               NX_NULL;

  /* Setup the driver state to not initialized.  */
  nx_driver_information.nx_driver_information_state =                NX_DRIVER_STATE_NOT_INITIALIZED;

  /* Setup the default packet pool for the driver's received packets.  */
  nx_driver_information.nx_driver_information_packet_pool_ptr = ip_ptr -> nx_ip_default_packet_pool;

  /* Get the rounded start pool start. */
  rounded_pool_start = nx_driver_information.nx_driver_information_packet_pool_ptr->nx_packet_pool_start;

  /* Calculate the address of payload. */
  payload_address = (CHAR *)((ALIGN_TYPE)rounded_pool_start + sizeof(NX_PACKET));

  /* Align the address of payload. */
  payload_address = (CHAR *)((((ALIGN_TYPE)payload_address + NX_PACKET_ALIGNMENT  - 1) / NX_PACKET_ALIGNMENT) * NX_PACKET_ALIGNMENT);

  /* Calculate the header size. */
  header_size = (ULONG)((ALIGN_TYPE)payload_address - (ALIGN_TYPE)rounded_pool_start);

  /* Clear the deferred events for the driver.  */
  nx_driver_information.nx_driver_information_deferred_events =       0;

  /* Call the hardware-specific ethernet controller initialization.  */
  status =  _nx_driver_hardware_initialize(driver_req_ptr);

  /* Determine if the request was successful.  */
  if (status == NX_SUCCESS)
  {

    /* Successful hardware initialization.  */

    /* Setup driver information to point to IP pointer.  */
    nx_driver_information.nx_driver_information_ip_ptr = driver_req_ptr -> nx_ip_driver_ptr;

    /* Setup the link maximum transfer unit. */
    interface_ptr -> nx_interface_ip_mtu_size =  NX_DRIVER_ETHERNET_MTU - NX_DRIVER_ETHERNET_FRAME_SIZE;

    /* Setup the physical address of this IP instance.  Increment the
    physical address lsw to simulate multiple nodes hanging on the
    ethernet.  */
    interface_ptr -> nx_interface_physical_address_msw = (ULONG)(GMACdev.mac_addr[0] << 8 |  ( GMACdev.mac_addr[1]));
    interface_ptr -> nx_interface_physical_address_lsw = (ULONG)(( GMACdev.mac_addr[2] << 24) | ( GMACdev.mac_addr[3] << 16) | ( GMACdev.mac_addr[4] << 8) | ( GMACdev.mac_addr[5]));

    /* Indicate to the IP software that IP to physical mapping
    is required.  */
    interface_ptr -> nx_interface_address_mapping_needed =  NX_TRUE;

    /* Move the driver's state to initialized.  */
    nx_driver_information.nx_driver_information_state = NX_DRIVER_STATE_INITIALIZED;

    /* Indicate successful initialize.  */
    driver_req_ptr -> nx_ip_driver_status =  NX_SUCCESS;
  }
  else
  {

    /* Initialization failed.  Indicate that the request failed.  */
    driver_req_ptr -> nx_ip_driver_status =   NX_DRIVER_ERROR;
  }
}


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _nx_driver_enable                                                   */
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Yuxin Zhou, Microsoft Corporation                                   */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function processing the initialize request. The processing     */
/*    in this function is generic. All ethernet controller logic is to    */
/*    be placed in _nx_driver_hardware_enable.                            */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    driver_req_ptr                        Driver command from the IP    */
/*                                            thread                      */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _nx_driver_hardware_enable            Process enable request        */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Driver entry function                                               */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Yuxin Zhou               Initial Version 6.0           */
/*  xx-xx-xxxx     Yuxin Zhou               Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/
static VOID  _nx_driver_enable(NX_IP_DRIVER *driver_req_ptr)
{
  NX_IP           *ip_ptr;
  UINT            status;

  /* Setup the IP pointer from the driver request.  */
  ip_ptr =  driver_req_ptr -> nx_ip_driver_ptr;

  /* See if we can honor the NX_LINK_ENABLE request.  */
  if (nx_driver_information.nx_driver_information_state < NX_DRIVER_STATE_INITIALIZED)
  {

    /* Mark the request as not successful.  */
    driver_req_ptr -> nx_ip_driver_status =  NX_DRIVER_ERROR;
    return;
  }

  /* Check if it is enabled by someone already */
  if (nx_driver_information.nx_driver_information_state >=  NX_DRIVER_STATE_LINK_ENABLED)
  {

    /* Yes, the request has already been made.  */
    driver_req_ptr -> nx_ip_driver_status =  NX_ALREADY_ENABLED;
    return;
  }

  /* Call hardware specific enable.  */
  status =  _nx_driver_hardware_enable(driver_req_ptr);

  /* Was the hardware enable successful?  */
  if (status == NX_SUCCESS)
  {

    /* Update the driver state to link enabled.  */
    nx_driver_information.nx_driver_information_state = NX_DRIVER_STATE_LINK_ENABLED;

    /* Mark request as successful.  */
    driver_req_ptr -> nx_ip_driver_status =  NX_SUCCESS;

    /* Mark the IP instance as link up.  */
    ip_ptr -> nx_ip_driver_link_up =  NX_TRUE;
  }
  else
  {

    /* Enable failed.  Indicate that the request failed.  */
    driver_req_ptr -> nx_ip_driver_status =   NX_DRIVER_ERROR;
  }
}


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _nx_driver_disable                                                  */
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Yuxin Zhou, Microsoft Corporation                                   */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function processing the disable request. The processing        */
/*    in this function is generic. All ethernet controller logic is to    */
/*    be placed in _nx_driver_hardware_disable.                           */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    driver_req_ptr                        Driver command from the IP    */
/*                                            thread                      */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _nx_driver_hardware_disable           Process disable request       */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Driver entry function                                               */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Yuxin Zhou               Initial Version 6.0           */
/*  xx-xx-xxxx     Yuxin Zhou               Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/
static VOID  _nx_driver_disable(NX_IP_DRIVER *driver_req_ptr)
{

  NX_IP           *ip_ptr;
  UINT            status;


  /* Setup the IP pointer from the driver request.  */
  ip_ptr =  driver_req_ptr -> nx_ip_driver_ptr;

  /* Check if the link is enabled.  */
  if (nx_driver_information.nx_driver_information_state !=  NX_DRIVER_STATE_LINK_ENABLED)
  {

    /* The link is not enabled, so just return an error.  */
    driver_req_ptr -> nx_ip_driver_status =  NX_DRIVER_ERROR;
    return;
  }

  /* Call hardware specific disable.  */
  status =  _nx_driver_hardware_disable(driver_req_ptr);

  /* Was the hardware disable successful?  */
  if (status == NX_SUCCESS)
  {

    /* Mark the IP instance as link down.  */
    ip_ptr -> nx_ip_driver_link_up =  NX_FALSE;

    /* Update the driver state back to initialized.  */
    nx_driver_information.nx_driver_information_state =  NX_DRIVER_STATE_INITIALIZED;

    /* Mark request as successful.  */
    driver_req_ptr -> nx_ip_driver_status =  NX_SUCCESS;
  }
  else
  {

    /* Disable failed, return an error.  */
    driver_req_ptr -> nx_ip_driver_status =  NX_DRIVER_ERROR;
  }
}


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _nx_driver_packet_send                                              */
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Yuxin Zhou, Microsoft Corporation                                   */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function processing the packet send request. The processing    */
/*    in this function is generic. All ethernet controller packet send    */
/*    logic is to be placed in _nx_driver_hardware_packet_send.           */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    driver_req_ptr                        Driver command from the IP    */
/*                                            thread                      */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _nx_driver_hardware_packet_send       Process packet send request   */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Driver entry function                                               */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Yuxin Zhou               Initial Version 6.0           */
/*  xx-xx-xxxx     Yuxin Zhou               Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/
static VOID  _nx_driver_packet_send(NX_IP_DRIVER *driver_req_ptr)
{

  NX_IP           *ip_ptr;
  NX_PACKET       *packet_ptr;
  ULONG           *ethernet_frame_ptr;
  UINT            status;


  /* Setup the IP pointer from the driver request.  */
  ip_ptr =  driver_req_ptr -> nx_ip_driver_ptr;

  /* Check to make sure the link is up.  */
  if (nx_driver_information.nx_driver_information_state != NX_DRIVER_STATE_LINK_ENABLED)
  {

    /* Inidate an unsuccessful packet send.  */
    driver_req_ptr -> nx_ip_driver_status =  NX_DRIVER_ERROR;

    /* Link is not up, simply free the packet.  */
    nx_packet_transmit_release(driver_req_ptr -> nx_ip_driver_packet);
    return;
  }

  /* Process driver send packet.  */

  /* Place the ethernet frame at the front of the packet.  */
  packet_ptr =  driver_req_ptr -> nx_ip_driver_packet;

  /* Adjust the prepend pointer.  */
  packet_ptr -> nx_packet_prepend_ptr =
    packet_ptr -> nx_packet_prepend_ptr - NX_DRIVER_ETHERNET_FRAME_SIZE;

  /* Adjust the packet length.  */
  packet_ptr -> nx_packet_length = packet_ptr -> nx_packet_length + NX_DRIVER_ETHERNET_FRAME_SIZE;

  /* Setup the ethernet frame pointer to build the ethernet frame.  Backup another 2
  * bytes to get 32-bit word alignment.  */
  ethernet_frame_ptr =  (ULONG *) (packet_ptr -> nx_packet_prepend_ptr - 2);

  /* Set up the hardware addresses in the Ethernet header. */
  *ethernet_frame_ptr       =  driver_req_ptr -> nx_ip_driver_physical_address_msw;
  *(ethernet_frame_ptr + 1) =  driver_req_ptr -> nx_ip_driver_physical_address_lsw;

  *(ethernet_frame_ptr + 2) =  (ip_ptr -> nx_ip_arp_physical_address_msw << 16) |
    (ip_ptr -> nx_ip_arp_physical_address_lsw >> 16);
  *(ethernet_frame_ptr + 3) =  (ip_ptr -> nx_ip_arp_physical_address_lsw << 16);

  /* Set up the frame type field in the Ethernet harder. */
  if ((driver_req_ptr -> nx_ip_driver_command == NX_LINK_ARP_SEND)||
      (driver_req_ptr -> nx_ip_driver_command == NX_LINK_ARP_RESPONSE_SEND))
  {

    *(ethernet_frame_ptr + 3) |= NX_DRIVER_ETHERNET_ARP;
  }
  else if(driver_req_ptr -> nx_ip_driver_command == NX_LINK_RARP_SEND)
  {

    *(ethernet_frame_ptr + 3) |= NX_DRIVER_ETHERNET_RARP;
  }

#ifdef FEATURE_NX_IPV6
  else if(packet_ptr -> nx_packet_ip_version == NX_IP_VERSION_V6)
  {

    *(ethernet_frame_ptr + 3) |= NX_DRIVER_ETHERNET_IPV6;
  }
#endif /* FEATURE_NX_IPV6 */

  else
  {

    *(ethernet_frame_ptr + 3) |= NX_DRIVER_ETHERNET_IP;
  }

  /* Endian swapping if NX_LITTLE_ENDIAN is defined.  */
  NX_CHANGE_ULONG_ENDIAN(*(ethernet_frame_ptr));
  NX_CHANGE_ULONG_ENDIAN(*(ethernet_frame_ptr + 1));
  NX_CHANGE_ULONG_ENDIAN(*(ethernet_frame_ptr + 2));
  NX_CHANGE_ULONG_ENDIAN(*(ethernet_frame_ptr + 3));

  /* Determine if the packet exceeds the driver's MTU.  */
  if (packet_ptr -> nx_packet_length > NX_DRIVER_ETHERNET_MTU)
  {

    /* This packet exceeds the size of the driver's MTU. Simply throw it away! */

    /* Remove the Ethernet header.  */
    NX_DRIVER_ETHERNET_HEADER_REMOVE(packet_ptr);

    /* Indicate an unsuccessful packet send.  */
    driver_req_ptr -> nx_ip_driver_status =  NX_DRIVER_ERROR;

    /* Link is not up, simply free the packet.  */
    nx_packet_transmit_release(packet_ptr);
    return;
  }

  /* Transmit the packet through the Ethernet controller low level access routine. */
  status = _nx_driver_hardware_packet_send(packet_ptr);

  /* Determine if there was an error.  */
  if (status != NX_SUCCESS)
  {

    /* Driver's hardware send packet routine failed to send the packet.  */

    /* Remove the Ethernet header.  */
    NX_DRIVER_ETHERNET_HEADER_REMOVE(packet_ptr);

    /* Indicate an unsuccessful packet send.  */
    driver_req_ptr -> nx_ip_driver_status =  NX_DRIVER_ERROR;

    /* Link is not up, simply free the packet.  */
    nx_packet_transmit_release(packet_ptr);
  }
  else
  {

    /* Set the status of the request.  */
    driver_req_ptr -> nx_ip_driver_status =  NX_SUCCESS;
  }
}


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _nx_driver_multicast_join                                           */
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Yuxin Zhou, Microsoft Corporation                                   */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function processing the multicast join request. The processing */
/*    in this function is generic. All ethernet controller multicast join */
/*    logic is to be placed in _nx_driver_hardware_multicast_join.        */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    driver_req_ptr                        Driver command from the IP    */
/*                                            thread                      */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _nx_driver_hardware_multicast_join    Process multicast join request*/
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Driver entry function                                               */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Yuxin Zhou               Initial Version 6.0           */
/*  xx-xx-xxxx     Yuxin Zhou               Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/
static VOID  _nx_driver_multicast_join(NX_IP_DRIVER *driver_req_ptr)
{

  UINT        status;


  /* Call hardware specific multicast join function. */
  status =  _nx_driver_hardware_multicast_join(driver_req_ptr);

  /* Determine if there was an error.  */
  if (status != NX_SUCCESS)
  {

    /* Indicate an unsuccessful request.  */
    driver_req_ptr -> nx_ip_driver_status =  NX_DRIVER_ERROR;
  }
  else
  {

    /* Indicate the request was successful.   */
    driver_req_ptr -> nx_ip_driver_status =  NX_SUCCESS;
  }
}


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _nx_driver_multicast_leave                                          */
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Yuxin Zhou, Microsoft Corporation                                   */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function processing the multicast leave request. The           */
/*    processing in this function is generic. All ethernet controller     */
/*    multicast leave logic is to be placed in                            */
/*    _nx_driver_hardware_multicast_leave.                                */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    driver_req_ptr                        Driver command from the IP    */
/*                                            thread                      */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _nx_driver_hardware_multicast_leave   Process multicast leave req   */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Driver entry function                                               */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Yuxin Zhou               Initial Version 6.0           */
/*  xx-xx-xxxx     Yuxin Zhou               Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/
static VOID  _nx_driver_multicast_leave(NX_IP_DRIVER *driver_req_ptr)
{

  UINT        status;


  /* Call hardware specific multicast leave function. */
  status =  _nx_driver_hardware_multicast_leave(driver_req_ptr);

  /* Determine if there was an error.  */
  if (status != NX_SUCCESS)
  {

    /* Indicate an unsuccessful request.  */
    driver_req_ptr -> nx_ip_driver_status =  NX_DRIVER_ERROR;
  }
  else
  {

    /* Indicate the request was successful.   */
    driver_req_ptr -> nx_ip_driver_status =  NX_SUCCESS;
  }
}


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _nx_driver_get_status                                               */
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Yuxin Zhou, Microsoft Corporation                                   */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function processing the get status request. The processing     */
/*    in this function is generic. All ethernet controller get status     */
/*    logic is to be placed in _nx_driver_hardware_get_status.            */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    driver_req_ptr                        Driver command from the IP    */
/*                                            thread                      */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _nx_driver_hardware_get_status        Process get status request    */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Driver entry function                                               */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Yuxin Zhou               Initial Version 6.0           */
/*  xx-xx-xxxx     Yuxin Zhou               Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/
static VOID  _nx_driver_get_status(NX_IP_DRIVER *driver_req_ptr)
{

  UINT        status;


  /* Call hardware specific get status function. */
  status =  _nx_driver_hardware_get_status(driver_req_ptr);

  /* Determine if there was an error.  */
  if (status != NX_SUCCESS)
  {

    /* Indicate an unsuccessful request.  */
    driver_req_ptr -> nx_ip_driver_status =  NX_DRIVER_ERROR;
  }
  else
  {

    /* Indicate the request was successful.   */
    driver_req_ptr -> nx_ip_driver_status =  NX_SUCCESS;
  }
}


#ifdef NX_ENABLE_INTERFACE_CAPABILITY
/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _nx_driver_capability_get                                           */
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Yuxin Zhou, Microsoft Corporation                                   */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function processing the get capability request.                */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    driver_req_ptr                        Driver command from the IP    */
/*                                            thread                      */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Driver entry function                                               */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Yuxin Zhou               Initial Version 6.0           */
/*  xx-xx-xxxx     Yuxin Zhou               Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/
static VOID  _nx_driver_capability_get(NX_IP_DRIVER *driver_req_ptr)
{

  /* Return the capability of the Ethernet controller.  */
  *(driver_req_ptr -> nx_ip_driver_return_ptr) = NX_DRIVER_CAPABILITY;

  /* Return the success status.  */
  driver_req_ptr -> nx_ip_driver_status =  NX_SUCCESS;
}


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _nx_driver_capability_set                                                  */
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Yuxin Zhou, Microsoft Corporation                                   */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function processing the set capability request.                */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    driver_req_ptr                        Driver command from the IP    */
/*                                            thread                      */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Driver entry function                                               */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Yuxin Zhou               Initial Version 6.0           */
/*  xx-xx-xxxx     Yuxin Zhou               Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/
static VOID  _nx_driver_capability_set(NX_IP_DRIVER *driver_req_ptr)
{

  UINT        status;


  /* Call hardware specific get status function. */
  status =  _nx_driver_hardware_capability_set(driver_req_ptr);

  /* Determine if there was an error.  */
  if (status != NX_SUCCESS)
  {

    /* Indicate an unsuccessful request.  */
    driver_req_ptr -> nx_ip_driver_status =  NX_DRIVER_ERROR;
  }
  else
  {

    /* Indicate the request was successful.   */
    driver_req_ptr -> nx_ip_driver_status =  NX_SUCCESS;
  }
}
#endif /* NX_ENABLE_INTERFACE_CAPABILITY */


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _nx_driver_deferred_processing                                      */
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    XC, Microsoft Corporation                                           */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function processing the deferred ISR action within the context */
/*    of the IP thread.                                                   */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    driver_req_ptr                        Driver command from the IP    */
/*                                            thread                      */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _nx_driver_packet_transmitted        Clean up after transmission    */
/*    _nx_driver_packet_received           Process a received packet      */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Driver entry function                                               */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Yuxin Zhou               Initial Version 6.0           */
/*  xx-xx-xxxx     Yuxin Zhou               Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/
#ifdef NX_DRIVER_ENABLE_DEFERRED
static VOID  _nx_driver_deferred_processing(NX_IP_DRIVER *driver_req_ptr)
{
  TX_INTERRUPT_SAVE_AREA

  ULONG       deferred_events;

  /* Disable interrupts.  */
  TX_DISABLE

    /* Pickup deferred events.  */
  deferred_events =  nx_driver_information.nx_driver_information_deferred_events;
  nx_driver_information.nx_driver_information_deferred_events =  0;

  /* Restore interrupts.  */
  TX_RESTORE

  /* Check for received packet.  */
  if(deferred_events & NX_DRIVER_DEFERRED_PACKET_RECEIVED)
  {
    /* Process received packet(s).  */
    _nx_driver_hardware_packet_received();
  }

  /* Mark request as successful.  */
  driver_req_ptr->nx_ip_driver_status =  NX_SUCCESS;
}
#endif  // #ifdef NX_DRIVER_ENABLE_DEFERRED


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _nx_driver_transfer_to_netx                                         */
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Yuxin Zhou, Microsoft Corporation                                   */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function processing incoming packets.  This routine would      */
/*    be called from the driver-specific receive packet processing        */
/*    function _nx_driver_hardware.                              */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    ip_ptr                                Pointer to IP protocol block  */
/*    packet_ptr                            Packet pointer                */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Error indication                                                    */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _nx_ip_packet_receive                 NetX IP packet receive        */
/*    _nx_ip_packet_deferred_receive        NetX IP packet receive        */
/*    _nx_arp_packet_deferred_receive       NetX ARP packet receive       */
/*    _nx_rarp_packet_deferred_receive      NetX RARP packet receive      */
/*    _nx_packet_release                    Release packet                */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    _nx_driver_hardware_packet_received   Driver packet receive function*/
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Yuxin Zhou               Initial Version 6.0           */
/*  xx-xx-xxxx     Yuxin Zhou               Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/
static VOID _nx_driver_transfer_to_netx(NX_IP *ip_ptr, NX_PACKET *packet_ptr)
{

  USHORT    packet_type;

  /* Set the interface for the incoming packet.  */
  packet_ptr -> nx_packet_ip_interface = nx_driver_information.nx_driver_information_interface;

  /* Pickup the packet header to determine where the packet needs to be
  sent.  */
  packet_type =  (USHORT)(((UINT) (*(packet_ptr -> nx_packet_prepend_ptr+12))) << 8) |
    ((UINT) (*(packet_ptr -> nx_packet_prepend_ptr+13)));

  /* Route the incoming packet according to its ethernet type.  */
  if (packet_type == NX_DRIVER_ETHERNET_IP || packet_type == NX_DRIVER_ETHERNET_IPV6)
  {
    /* Note:  The length reported by some Ethernet hardware includes
    bytes after the packet as well as the Ethernet header.  In some
    cases, the actual packet length after the Ethernet header should
    be derived from the length in the IP header (lower 16 bits of
    the first 32-bit word).  */

    /* Clean off the Ethernet header.  */
    packet_ptr -> nx_packet_prepend_ptr += NX_DRIVER_ETHERNET_FRAME_SIZE;

    /* Adjust the packet length.  */
    packet_ptr -> nx_packet_length -= NX_DRIVER_ETHERNET_FRAME_SIZE;

    /* Route to the ip receive function.  */
#ifdef NX_DRIVER_ENABLE_DEFERRED
    _nx_ip_packet_deferred_receive(ip_ptr, packet_ptr);
#else  // #ifdef NX_DRIVER_ENABLE_DEFERRED
    _nx_ip_packet_receive(ip_ptr, packet_ptr);
#endif  // #ifdef NX_DRIVER_ENABLE_DEFERRED
  }
  else if (packet_type == NX_DRIVER_ETHERNET_ARP)
  {

    /* Clean off the Ethernet header.  */
    packet_ptr -> nx_packet_prepend_ptr += NX_DRIVER_ETHERNET_FRAME_SIZE;

    /* Adjust the packet length.  */
    packet_ptr -> nx_packet_length -= NX_DRIVER_ETHERNET_FRAME_SIZE;

    /* Route to the ARP receive function.  */
    _nx_arp_packet_deferred_receive(ip_ptr, packet_ptr);
  }
  else if (packet_type == NX_DRIVER_ETHERNET_RARP)
  {

    /* Clean off the Ethernet header.  */
    packet_ptr -> nx_packet_prepend_ptr += NX_DRIVER_ETHERNET_FRAME_SIZE;

    /* Adjust the packet length.  */
    packet_ptr -> nx_packet_length -= NX_DRIVER_ETHERNET_FRAME_SIZE;

    /* Route to the RARP receive function.  */
    _nx_rarp_packet_deferred_receive(ip_ptr, packet_ptr);
  }
  else
  {
    /* Invalid ethernet header... release the packet.  */
    nx_packet_release(packet_ptr);
  }
}


/****** DRIVER SPECIFIC ****** Start of part/vendor specific internal driver functions.  */

/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _nx_driver_hardware_initialize                                      */
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Yuxin Zhou, Microsoft Corporation                                   */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function processes hardware-specific initialization.           */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    driver_req_ptr                        Driver request pointer        */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    status                                [NX_SUCCESS|NX_DRIVER_ERROR]  */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    ETH_BSP_Config                        Configure Ethernet            */
/*    ETH_MACAddressConfig                  Setup MAC address             */
/*    ETH_DMARxDescReceiveITConfig          Enable receive descriptors    */
/*    nx_packet_allocate                    Allocate receive packet(s)    */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    _nx_driver_initialize                 Driver initialize processing  */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Yuxin Zhou               Initial Version 6.0           */
/*  xx-xx-xxxx     Yuxin Zhou               Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/
static UINT  _nx_driver_hardware_initialize(NX_IP_DRIVER *driver_req_ptr)
{
  /* Default to successful return.  */
  driver_req_ptr -> nx_ip_driver_status = NX_SUCCESS;

  /* Setup indices.  */
  nx_driver_information.nx_driver_information_receive_current_index = 0;
  nx_driver_information.nx_driver_information_transmit_current_index = 0;
  nx_driver_information.nx_driver_information_transmit_release_index = 0;

  /* Clear the number of buffers in use counter.  */
  nx_driver_information.nx_driver_information_number_of_transmit_buffers_in_use = 0;

  /* Make sure there are receive packets... otherwise, return an error.  */
  if (nx_driver_information.nx_driver_information_packet_pool_ptr == NULL)
  {

    /* There must be receive packets. If not, return an error!  */
    return(NX_DRIVER_ERROR);
  }

#ifdef NX_DRIVER_ETH_HW_IP_INIT
  nx_eth_init();
#endif /* NX_DRIVER_ETH_HW_IP_INIT */

  SYS_ResetModule(EMAC0_RST);
  
  /* Enable EMAC module clock */
  CLK_EnableModuleClock(EMAC0_MODULE);
      
  /* Configure pins for EMAC module */
  SET_EMAC0_RMII_MDC_PE8();
  SET_EMAC0_RMII_MDIO_PE9();
  SET_EMAC0_RMII_TXD0_PE10();
  SET_EMAC0_RMII_TXD1_PE11();
  SET_EMAC0_RMII_TXEN_PE12();
  SET_EMAC0_RMII_REFCLK_PC8();
  SET_EMAC0_RMII_RXD0_PC7();
  SET_EMAC0_RMII_RXD1_PC6();
  SET_EMAC0_RMII_CRSDV_PA7();
  SET_EMAC0_RMII_RXERR_PA6();
  SET_EMAC0_PPS_PB6();

  static uint8_t macaddr[] = { 0x2C, 0x77, 0xD4, 0xDA, 0xFE, 0x07 }; // FIXME!!
  /* Attach the device to MAC struct This will configure all the required base addresses
    such as Mac base, configuration base, phy base address(out of 32 possible phys) */
  synopGMAC_attach(&GMACdev, (EMAC_BASE + MACBASE), (EMAC_BASE + DMABASE), DEFAULT_PHY_BASE, macaddr);

  synopGMAC_reset(&GMACdev);

  /* Lets read the version of ip in to device structure */
  synopGMAC_read_version(&GMACdev);

  memcpy((void*)&GMACdev.mac_addr[0], (void*)&macaddr[0], 6);

  /* Check for Phy initialization */
  if(SystemCoreClock >= 250000000)
      synopGMAC_set_mdc_clk_div(&GMACdev, GmiiCsrClk5);
  else if(SystemCoreClock >= 150000000)
      synopGMAC_set_mdc_clk_div(&GMACdev, GmiiCsrClk4);
  else if(SystemCoreClock >= 100000000)
      synopGMAC_set_mdc_clk_div(&GMACdev, GmiiCsrClk1);
  else if(SystemCoreClock >= 60000000)
      synopGMAC_set_mdc_clk_div(&GMACdev, GmiiCsrClk0);
  else if(SystemCoreClock >= 35000000)
      synopGMAC_set_mdc_clk_div(&GMACdev, GmiiCsrClk3);
  else
      synopGMAC_set_mdc_clk_div(&GMACdev, GmiiCsrClk2);
  GMACdev.ClockDivMdc = synopGMAC_get_mdc_clk_div(&GMACdev);    

  if(_nx_mii_check_phy_init(&GMACdev) < 0) {
      printf("emac:: Init PHY FAIL.\n");
      printf("\n??? Check the PHY device...\n");
      while(1) {}
  }

  /* Set up the tx and rx descriptor queue/ring */
  synopGMAC_setup_tx_desc_queue(&GMACdev, &tx_desc[0], TRANSMIT_DESC_SIZE, RINGMODE);
  synopGMAC_init_tx_desc_base(&GMACdev);	// Program the transmit descriptor base address in to DmaTxBase addr
//  printf("DmaTxBaseAddr = %08x\n", synopGMACReadReg(GMACdev.DmaBase, DmaTxBaseAddr));

  synopGMAC_setup_rx_desc_queue(&GMACdev, &rx_desc[0], RECEIVE_DESC_SIZE, RINGMODE);
  synopGMAC_init_rx_desc_base(&GMACdev); // Program the transmit descriptor base address in to DmaTxBase addr
//  printf("DmaTxBaseAddr = %08x\n", synopGMACReadReg(GMACdev.DmaBase, DmaRxBaseAddr));

  synopGMAC_dma_bus_mode_init(&GMACdev, (DmaBurstLength32 | DmaDescriptorSkip0 | DmaDescriptor8Words));
  synopGMAC_dma_control_init(&GMACdev, (DmaStoreAndForward | DmaTxSecondFrame| DmaRxThreshCtrl128));

  /* Initialize the mac interface */
  synopGMAC_mac_init(&GMACdev);
  synopGMAC_promisc_enable(&GMACdev);

  synopGMAC_pause_control(&GMACdev); // This enables the pause control in Full duplex mode of operation

#if defined(NX_ENABLE_INTERFACE_CAPABILITY)
  /* IPC Checksum offloading is enabled for this driver. Should only be used if Full Ip checksumm offload engine is configured in the hardware */
  synopGMAC_enable_rx_chksum_offload(&GMACdev);    // Enable the offload engine in the receive path
  synopGMAC_rx_tcpip_chksum_drop_enable(&GMACdev); // This is default configuration, DMA drops the packets if error in encapsulated ethernet payload
#endif

  for(size_t i=0; i<RECEIVE_DESC_SIZE; i ++) {
      memset(&rx_buf[i], 0, sizeof(PKT_FRAME_T));
      synopGMAC_set_rx_qptr(&GMACdev, (u32)&rx_buf[i], PKT_FRAME_BUF_SIZE, 0);
  }

  /* Enable interrupt */
  synopGMAC_clear_interrupt(&GMACdev);
  synopGMAC_disable_interrupt_all(&GMACdev);
  synopGMAC_enable_interrupt(&GMACdev, DmaIntEnable);

  /* Enable DMA */
  synopGMAC_enable_dma_rx(&GMACdev);
  synopGMAC_enable_dma_tx(&GMACdev);

  synopGMAC_set_mac_addr(&GMACdev, GmacAddr0High, GmacAddr0Low, &GMACdev.mac_addr[0]);

  if(GMACdev.Speed == SPEED10)
      synopGMAC_set_mode(&GMACdev, 2); // 1: 100Mbps, 2: 10Mbps
  else
      synopGMAC_set_mode(&GMACdev, 1); // 1: 100Mbps, 2: 10Mbps

  /* Clear the number of buffers in use counter.  */
  nx_driver_information.nx_driver_information_multicast_count = 0;

  synopGMACdeviceInit = true;

  /* Return success!  */
  return(NX_SUCCESS);
}


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _nx_driver_hardware_enable                                          */
/*                                                            6.1         */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Yuxin Zhou, Microsoft Corporation                                   */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function processes hardware-specific link enable requests.     */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    driver_req_ptr                        Driver request pointer        */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    status                                [NX_SUCCESS|NX_DRIVER_ERROR]  */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    ETH_Start                             Start Ethernet operation      */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    _nx_driver_enable                     Driver link enable processing */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Yuxin Zhou               Initial Version 6.0           */
/*  xx-xx-xxxx     Yuxin Zhou               Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/
static UINT  _nx_driver_hardware_enable(NX_IP_DRIVER *driver_req_ptr)
{
    NVIC_EnableIRQ(EMAC0_TXRX_IRQn);

    /* Return success!  */
    return(NX_SUCCESS);
}


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _nx_driver_hardware_disable                                         */
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Yuxin Zhou, Microsoft Corporation                                   */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function processes hardware-specific link disable requests.    */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    driver_req_ptr                        Driver request pointer        */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    status                                [NX_SUCCESS|NX_DRIVER_ERROR]  */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    ETH_MACTransmissionCmd                Disable transmit              */
/*    ETH_FlushTransmitFIFO                 Flush transmit FIFO           */
/*    ETH_MACReceptionCmd                   Disable receive               */
/*    ETH_DMATransmissionCmd                Stop DMA transmission         */
/*    ETH_DMAReceptionCmd                   Stop DMA reception            */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    _nx_driver_disable                    Driver link disable processing*/
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Yuxin Zhou               Initial Version 6.0           */
/*  xx-xx-xxxx     Yuxin Zhou               Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/
static UINT  _nx_driver_hardware_disable(NX_IP_DRIVER *driver_req_ptr)
{
    NVIC_DisableIRQ(EMAC0_TXRX_IRQn);

    /* Return success!  */
    return(NX_SUCCESS);
}


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _nx_driver_hardware_packet_send                                     */
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Yuxin Zhou, Microsoft Corporation                                   */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function processes hardware-specific packet send requests.     */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    packet_ptr                            Pointer to packet to send     */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    status                                [NX_SUCCESS|NX_DRIVER_ERROR]  */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    [_nx_driver_transmit_packet_enqueue]  Optional internal transmit    */
/*                                            packet queue routine        */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    _nx_driver_packet_send                Driver packet send processing */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Yuxin Zhou               Initial Version 6.0           */
/*  xx-xx-xxxx     Yuxin Zhou               Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/

static UINT _nx_driver_hardware_packet_send(NX_PACKET *packet_ptr)
{
    if (!synopGMACdeviceInit) {
        return NX_DRIVER_ERROR;
    }

    if (synopGMAC_is_desc_owned_by_dma(GMACdev.TxNextDesc)) {
        return(NX_DRIVER_ERROR);
    }

  for (NX_PACKET *pktIdx = packet_ptr; pktIdx != NX_NULL; pktIdx = pktIdx -> nx_packet_next) {

      uint8_t *buf = pktIdx->nx_packet_prepend_ptr;
      size_t len = (pktIdx -> nx_packet_append_ptr - pktIdx->nx_packet_prepend_ptr);

      if (buf == 0 || len == 0) {
          continue;
      }

#if defined (__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
      SCB_CleanDCache_by_Addr((uint32_t*)(pktIdx->nx_packet_data_start), pktIdx->nx_packet_data_end - pktIdx->nx_packet_data_start);
#endif  // #if defined (__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)

      int offload = 0;
#ifdef NX_ENABLE_INTERFACE_CAPABILITY
      if (packet_ptr->nx_packet_interface_capability_flag & (NX_INTERFACE_CAPABILITY_TCP_TX_CHECKSUM |
                                                             NX_INTERFACE_CAPABILITY_UDP_TX_CHECKSUM |
                                                             NX_INTERFACE_CAPABILITY_ICMPV4_TX_CHECKSUM |
                                                             NX_INTERFACE_CAPABILITY_ICMPV6_TX_CHECKSUM)) {
          offload = 1; // FIXME!!
      }
      else if (packet_ptr->nx_packet_interface_capability_flag & NX_INTERFACE_CAPABILITY_IPV4_TX_CHECKSUM) {
          offload = 1; // FIXME!!
      }
#else
      offload = 0;
#endif /* NX_ENABLE_INTERFACE_CAPABILITY */
 
      TX_INTERRUPT_SAVE_AREA
      TX_DISABLE

      u32 index = GMACdev.TxNext;
      memcpy(&tx_buf[index].au8Buf[0], buf, len);

      if (synopGMAC_xmit_frames(&GMACdev, &tx_buf[index].au8Buf[0], len, offload, 0) < 0 ) {

          TX_RESTORE

          return(NX_DRIVER_ERROR);
      }

      TX_RESTORE
  }


  /* Remove the Ethernet header.  */
  NX_DRIVER_ETHERNET_HEADER_REMOVE(packet_ptr);

  /* Free the packet.  */
  nx_packet_transmit_release(packet_ptr);


  return(NX_SUCCESS);
}

/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _nx_driver_hardware_multicast_join                                  */
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Yuxin Zhou, Microsoft Corporation                                   */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function processes hardware-specific multicast join requests.  */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    driver_req_ptr                        Driver request pointer        */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    status                                [NX_SUCCESS|NX_DRIVER_ERROR]  */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    _nx_driver_multicast_join             Driver multicast join         */
/*                                            processing                  */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Yuxin Zhou               Initial Version 6.0           */
/*  xx-xx-xxxx     Yuxin Zhou               Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/
static UINT  _nx_driver_hardware_multicast_join(NX_IP_DRIVER *driver_req_ptr)
{
    if (!synopGMACdeviceInit) {
        return NX_DRIVER_ERROR;
    }

    /* Increase the multicast count.  */
    nx_driver_information.nx_driver_information_multicast_count++;

    /* Enable multicast frame reception.  */
    synopGMAC_multicast_enable(&GMACdev);

    /* Return success.  */
    return(NX_SUCCESS);
}


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _nx_driver_hardware_multicast_leave                                 */
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Yuxin Zhou, Microsoft Corporation                                   */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function processes hardware-specific multicast leave requests. */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    driver_req_ptr                        Driver request pointer        */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    status                                [NX_SUCCESS|NX_DRIVER_ERROR]*/
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    _nx_driver_multicast_leave            Driver multicast leave        */
/*                                            processing                  */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Yuxin Zhou               Initial Version 6.0           */
/*  xx-xx-xxxx     Yuxin Zhou               Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/
static UINT  _nx_driver_hardware_multicast_leave(NX_IP_DRIVER *driver_req_ptr)
{
    if (!synopGMACdeviceInit) {
        return NX_DRIVER_ERROR;
    }

    /* Decrease the multicast count.  */
    nx_driver_information.nx_driver_information_multicast_count--;

    /* If multicast count reaches zero, disable multicast frame reception.  */
    if (nx_driver_information.nx_driver_information_multicast_count == 0) {
        synopGMAC_multicast_disable(&GMACdev);
    }

    /* Return success.  */
    return(NX_SUCCESS);
}


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _nx_driver_hardware_get_status                                      */
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Yuxin Zhou, Microsoft Corporation                                   */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function processes hardware-specific get status requests.      */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    driver_req_ptr                        Driver request pointer        */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    status                                [NX_SUCCESS|NX_DRIVER_ERROR]*/
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    _nx_driver_get_status                 Driver get status processing  */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Yuxin Zhou               Initial Version 6.0           */
/*  xx-xx-xxxx     Yuxin Zhou               Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/
static UINT  _nx_driver_hardware_get_status(NX_IP_DRIVER *driver_req_ptr)
{
    INT PHYLinkState;

    if (!synopGMACdeviceInit) {
        return NX_DRIVER_ERROR;
    }

    /* Get link status. */
    PHYLinkState = _nx_mii_link_ok(&GMACdev);

    /* Check link status. */
    if(PHYLinkState <= 0) {
        /* Update Link status if physical link is down. */
        *(driver_req_ptr->nx_ip_driver_return_ptr) = NX_FALSE;
    } else {
        /* Update Link status if physical link is up. */
        *(driver_req_ptr->nx_ip_driver_return_ptr) = NX_TRUE;
    }

    /* Return success. */
    return NX_SUCCESS;
}

#ifdef NX_ENABLE_INTERFACE_CAPABILITY
/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _nx_driver_hardware_capability_set                                  */
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Yuxin Zhou, Microsoft Corporation                                   */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function processes hardware-specific capability set requests.  */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    driver_req_ptr                         Driver request pointer       */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    status                                [NX_SUCCESS|NX_DRIVER_ERROR]  */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    _nx_driver_capability_set             Capability set processing     */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Yuxin Zhou               Initial Version 6.0           */
/*  xx-xx-xxxx     Yuxin Zhou               Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/
static UINT _nx_driver_hardware_capability_set(NX_IP_DRIVER *driver_req_ptr)
{
  return NX_SUCCESS;
}
#endif /* NX_ENABLE_INTERFACE_CAPABILITY */

/****** DRIVER SPECIFIC ****** Start of part/vendor specific internal driver functions.  */

/* Generic MII registers. */
#define MII_BMCR            0x00        /* Basic mode control register */
#define MII_BMSR            0x01        /* Basic mode status register  */
#define MII_PHYSID1         0x02        /* PHYS ID 1                   */
#define MII_PHYSID2         0x03        /* PHYS ID 2                   */
#define MII_ADVERTISE       0x04        /* Advertisement control reg   */
#define MII_LPA             0x05        /* Link partner ability reg    */
#define MII_EXPANSION       0x06        /* Expansion register          */
#define MII_CTRL1000        0x09        /* 1000BASE-T control          */
#define MII_STAT1000        0x0a        /* 1000BASE-T status           */
#define MII_ESTATUS         0x0f        /* Extended Status             */
#define MII_DCOUNTER        0x12        /* Disconnect counter          */
#define MII_FCSCOUNTER      0x13        /* False carrier counter       */
#define MII_NWAYTEST        0x14        /* N-way auto-neg test reg     */
#define MII_RERRCOUNTER     0x15        /* Receive error counter       */
#define MII_SREVISION       0x16        /* Silicon revision            */
#define MII_RESV1           0x17        /* Reserved...                 */
#define MII_LBRERROR        0x18        /* Lpback, rx, bypass error    */
#define MII_PHYADDR         0x19        /* PHY address                 */
#define MII_RESV2           0x1a        /* Reserved...                 */
#define MII_TPISTATUS       0x1b        /* TPI status for 10mbps       */
#define MII_NCONFIG         0x1c        /* Network interface config    */

/* Basic mode control register. */
#define BMCR_RESV               0x003f  /* Unused...                   */
#define BMCR_SPEED1000          0x0040  /* MSB of Speed (1000)         */
#define BMCR_CTST               0x0080  /* Collision test              */
#define BMCR_FULLDPLX           0x0100  /* Full duplex                 */
#define BMCR_ANRESTART          0x0200  /* Auto negotiation restart    */
#define BMCR_ISOLATE            0x0400  /* Disconnect DP83840 from MII */
#define BMCR_PDOWN              0x0800  /* Powerdown the DP83840       */
#define BMCR_ANENABLE           0x1000  /* Enable auto negotiation     */
#define BMCR_SPEED100           0x2000  /* Select 100Mbps              */
#define BMCR_LOOPBACK           0x4000  /* TXD loopback bits           */
#define BMCR_RESET              0x8000  /* Reset the DP83840           */

/* Basic mode status register. */
#define BMSR_ERCAP              0x0001  /* Ext-reg capability          */
#define BMSR_JCD                0x0002  /* Jabber detected             */
#define BMSR_LSTATUS            0x0004  /* Link status                 */
#define BMSR_ANEGCAPABLE        0x0008  /* Able to do auto-negotiation */
#define BMSR_RFAULT             0x0010  /* Remote fault detected       */
#define BMSR_ANEGCOMPLETE       0x0020  /* Auto-negotiation complete   */
#define BMSR_RESV               0x00c0  /* Unused...                   */
#define BMSR_ESTATEN            0x0100  /* Extended Status in R15 */
#define BMSR_100FULL2           0x0200  /* Can do 100BASE-T2 HDX */
#define BMSR_100HALF2           0x0400  /* Can do 100BASE-T2 FDX */
#define BMSR_10HALF             0x0800  /* Can do 10mbps, half-duplex  */
#define BMSR_10FULL             0x1000  /* Can do 10mbps, full-duplex  */
#define BMSR_100HALF            0x2000  /* Can do 100mbps, half-duplex */
#define BMSR_100FULL            0x4000  /* Can do 100mbps, full-duplex */
#define BMSR_100BASE4           0x8000  /* Can do 100mbps, 4k packets  */

/* Advertisement control register. */
#define ADVERTISE_SLCT          0x001f  /* Selector bits               */
#define ADVERTISE_CSMA          0x0001  /* Only selector supported     */
#define ADVERTISE_10HALF        0x0020  /* Try for 10mbps half-duplex  */
#define ADVERTISE_1000XFULL     0x0020  /* Try for 1000BASE-X full-duplex */
#define ADVERTISE_10FULL        0x0040  /* Try for 10mbps full-duplex  */
#define ADVERTISE_1000XHALF     0x0040  /* Try for 1000BASE-X half-duplex */
#define ADVERTISE_100HALF       0x0080  /* Try for 100mbps half-duplex */
#define ADVERTISE_1000XPAUSE    0x0080  /* Try for 1000BASE-X pause    */
#define ADVERTISE_100FULL       0x0100  /* Try for 100mbps full-duplex */
#define ADVERTISE_1000XPSE_ASYM 0x0100  /* Try for 1000BASE-X asym pause */
#define ADVERTISE_100BASE4      0x0200  /* Try for 100mbps 4k packets  */
#define ADVERTISE_PAUSE_CAP     0x0400  /* Try for pause               */
#define ADVERTISE_PAUSE_ASYM    0x0800  /* Try for asymetric pause     */
#define ADVERTISE_RESV          0x1000  /* Unused...                   */
#define ADVERTISE_RFAULT        0x2000  /* Say we can detect faults    */
#define ADVERTISE_LPACK         0x4000  /* Ack link partners response  */
#define ADVERTISE_NPAGE         0x8000  /* Next page bit               */

#define ADVERTISE_FULL (ADVERTISE_100FULL | ADVERTISE_10FULL | ADVERTISE_CSMA)
#define ADVERTISE_ALL (ADVERTISE_10HALF | ADVERTISE_10FULL | ADVERTISE_100HALF | ADVERTISE_100FULL)

/* Indicates what features are advertised by the interface. */
#define ADVERTISED_10baseT_Half     (1 << 0)
#define ADVERTISED_10baseT_Full     (1 << 1)
#define ADVERTISED_100baseT_Half    (1 << 2)
#define ADVERTISED_100baseT_Full    (1 << 3)
#define ADVERTISED_1000baseT_Half   (1 << 4)
#define ADVERTISED_1000baseT_Full   (1 << 5)
#define ADVERTISED_Autoneg          (1 << 6)
#define ADVERTISED_TP               (1 << 7)
#define ADVERTISED_AUI              (1 << 8)
#define ADVERTISED_MII              (1 << 9)
#define ADVERTISED_FIBRE            (1 << 10)
#define ADVERTISED_BNC              (1 << 11)
#define ADVERTISED_10000baseT_Full  (1 << 12)
#define ADVERTISED_Pause            (1 << 13)
#define ADVERTISED_Asym_Pause       (1 << 14)

/* Link partner ability register. */
#define LPA_SLCT                0x001f  /* Same as advertise selector  */
#define LPA_10HALF              0x0020  /* Can do 10mbps half-duplex   */
#define LPA_1000XFULL           0x0020  /* Can do 1000BASE-X full-duplex */
#define LPA_10FULL              0x0040  /* Can do 10mbps full-duplex   */
#define LPA_1000XHALF           0x0040  /* Can do 1000BASE-X half-duplex */
#define LPA_100HALF             0x0080  /* Can do 100mbps half-duplex  */
#define LPA_1000XPAUSE          0x0080  /* Can do 1000BASE-X pause     */
#define LPA_100FULL             0x0100  /* Can do 100mbps full-duplex  */
#define LPA_1000XPAUSE_ASYM     0x0100  /* Can do 1000BASE-X pause asym*/
#define LPA_100BASE4            0x0200  /* Can do 100mbps 4k packets   */
#define LPA_PAUSE_CAP           0x0400  /* Can pause                   */
#define LPA_PAUSE_ASYM          0x0800  /* Can pause asymetrically     */
#define LPA_RESV                0x1000  /* Unused...                   */
#define LPA_RFAULT              0x2000  /* Link partner faulted        */
#define LPA_LPACK               0x4000  /* Link partner acked us       */
#define LPA_NPAGE               0x8000  /* Next page bit               */

#define LPA_DUPLEX              (LPA_10FULL | LPA_100FULL)
#define LPA_100                 (LPA_100FULL | LPA_100HALF | LPA_100BASE4)

static int32_t _nx_mii_mdio_read(synopGMACdevice *gmacdev, uint16_t reg, uint16_t *val) {
    return synopGMAC_read_phy_reg((u32)gmacdev->MacBase, gmacdev->PhyBase, reg, val);
}

static int32_t _nx_mii_mdio_write(synopGMACdevice *gmacdev, uint16_t reg, uint16_t val) {
    return synopGMAC_write_phy_reg((u32)gmacdev->MacBase, gmacdev->PhyBase, reg, val);
}

static int32_t _nx_mii_link_ok(synopGMACdevice *gmacdev) {
    uint16_t value;
    int32_t ret = -1;
    
    /* first, a dummy read, needed to latch some MII phys */
    _nx_mii_mdio_read(gmacdev, MII_BMSR, &value);    
    ret = _nx_mii_mdio_read(gmacdev, MII_BMSR, &value);
    if(ret) {
        return ret;
    }
    
    if(value & BMSR_LSTATUS) {
        return 1;
    }
    
    return 0;
}

static uint16_t _nx_mii_nway_result(uint32_t negotiated) {
    uint16_t ret;

    if (negotiated & LPA_100FULL)
        ret = LPA_100FULL;
    else if (negotiated & LPA_100BASE4)
        ret = LPA_100BASE4;
    else if (negotiated & LPA_100HALF)
        ret = LPA_100HALF;
    else if (negotiated & LPA_10FULL)
        ret = LPA_10FULL;
    else
        ret = LPA_10HALF;

    return ret;
}

static int32_t _nx_mii_ethtool_gset(synopGMACdevice *gmacdev, uint8_t reset) {
    int32_t ret = -1;
    uint16_t val, lpa;
    volatile int32_t loop_count;
    
    gmacdev->LinkState = LINKDOWN;
    
    if(reset) {
        // perform PHY reset
        do {
            ret = _nx_mii_mdio_write(gmacdev, MII_BMCR, BMCR_RESET);
            if(ret < 0) {
                break;
            }
            
            loop_count = 10000;
            while(loop_count-- > 0) {
                ret = _nx_mii_mdio_read(gmacdev, MII_BMCR, &val);
                if(ret < 0)
                    break;
                if((val & BMCR_RESET) == 0)
                    break;
            }
            if((ret < 0) || (loop_count < 0)) {
                ret = -ESYNOPGMACPHYERR;
                break;
            }

            ret = _nx_mii_mdio_write(gmacdev, MII_ADVERTISE, (ADVERTISE_FULL | ADVERTISE_ALL));
            if(ret < 0) {
                break;
            }
            ret = _nx_mii_mdio_read(gmacdev, MII_BMCR, &val);
            if(ret < 0) {
                break;
            }
            ret = _nx_mii_mdio_write(gmacdev, MII_BMCR, (val | BMCR_ANRESTART));
            if(ret < 0) {
                break;
            }
        } while(0);
        
        if(ret < 0) {
            printf("mii:: Reset PHY, FAIL.\n");
            return -ESYNOPGMACPHYERR;
        }
        
        printf("mii:: Reset PHY, PASS.\n");
    }
    
    do {        
        loop_count = 200000;
        while(loop_count-- > 0) {
            ret = _nx_mii_mdio_read(gmacdev, MII_BMSR, &val);
            if(ret < 0)
                break;
            if((val & (BMSR_LSTATUS | BMSR_ANEGCOMPLETE)) == (BMSR_LSTATUS | BMSR_ANEGCOMPLETE))
                break;
        }
        if((ret < 0) || (loop_count < 0)) {
            gmacdev->DuplexMode = 0;
            gmacdev->Speed      = 0;
            ret = -ESYNOPGMACPHYERR;
            break;
        }
                
        ret = _nx_mii_mdio_read(gmacdev, MII_LPA, &lpa);
        if(ret < 0) {
            break;
        }
    } while(0);
    
    if(ret < 0) {
        printf("mii:: Read MII_BMSR status, FAIL.\n");
        return -ESYNOPGMACPHYERR;
    }    
    
    gmacdev->LinkState = LINKUP;
    
    val = _nx_mii_nway_result(lpa);
    if(val & LPA_100FULL) {
        printf("mii:: 100M FULLDUPLEX\n");
        gmacdev->DuplexMode = FULLDUPLEX;
        gmacdev->Speed      = SPEED100;
    } else if(val & LPA_100HALF) {
       printf("mii:: 100M HALFDUPLEX\n");
        gmacdev->DuplexMode = HALFDUPLEX;
        gmacdev->Speed      = SPEED100;
    } else if(val & LPA_10FULL) {
        printf("mii:: 10M FULLDUPLEX\n");
        gmacdev->DuplexMode = FULLDUPLEX;
        gmacdev->Speed      = SPEED10;
    } else if(val & LPA_10HALF) {
        printf("mii:: 10M HALFDUPLEX\n");
        gmacdev->DuplexMode = HALFDUPLEX;
        gmacdev->Speed      = SPEED10;
    } else {
        printf("mii:: 100M FULLDUPLEX (Default: LPA 0x%x)\n", val);
        gmacdev->DuplexMode = FULLDUPLEX;
        gmacdev->Speed      = SPEED100;
    }

    return 0;
}

static int32_t _nx_mii_check_phy_init(synopGMACdevice *gmacdev) {
    int32_t ret = -1;
    
    ret = _nx_mii_link_ok(gmacdev); 
    if(ret < 0) {
        return ret;
    }
    
    if(ret == LINKDOWN) {
        gmacdev->DuplexMode = 0;
        gmacdev->Speed      = 0;
        gmacdev->LinkState  = 0;
        gmacdev->LoopBackMode = 0;
        
        return ret;
    } else {
        _nx_mii_ethtool_gset(gmacdev, 1);
        
        return (gmacdev->Speed | (gmacdev->DuplexMode << 4));
    }
}

void EMAC0_IRQHandler(void) {
    u32 interrupt, dma_status_reg;
    s32 status;
    u32 u32GmacIntSts;
    u32 u32GmacDmaIE = DmaIntEnable;

#define LED_INIT() (PH->MODE = ((PH->MODE & (~(0x3ful << 4 * 2))) | (0x15ul << 4 * 2)))
#define LED_RED PH4
#define LED_YELLOW PH5
#define LED_GREEN PH6

    static bool init = false;
    if (!init) {
      init = true;
      /* Enable GPIO PH to control LED */
      CLK->AHBCLK0 |= CLK_AHBCLK0_GPHCKEN_Msk;
      LED_INIT();
      LED_YELLOW = 0;
      LED_RED = 0;
      LED_GREEN = 0;
    }

    // Check GMAC interrupt
    u32GmacIntSts = synopGMACReadReg(GMACdev.MacBase, GmacInterruptStatus);
    if (u32GmacIntSts & GmacTSIntSts) {
        GMACdev.synopGMACNetStats.ts_int = 1;
        status = synopGMACReadReg(GMACdev.MacBase, GmacTSStatus);
        if (!(status & (1 << 1)))
        {
//            printf("TS alarm flag not set??\n");
        }
        else
        {
//            printf("TS alarm!!!!!!!!!!!!!!!!\n");
        }
    }
  
    synopGMACWriteReg(GMACdev.MacBase, GmacInterruptStatus, u32GmacIntSts);

    dma_status_reg = synopGMACReadReg(GMACdev.DmaBase, DmaStatus);
    if (dma_status_reg == 0) {
//        printf("dma_status ==0 \n");
        return;
    }

    if (dma_status_reg & GmacPmtIntr) {
//        printf("%s:: Interrupt due to PMT module\n", __func__);
        synopGMAC_powerup_mac(&GMACdev);
    }

    if (dma_status_reg & GmacLineIntfIntr) {
//        printf("%s:: Interrupt due to GMAC LINE module\n", __func__);
        if (synopGMACReadReg(GMACdev.MacBase, GmacInterruptStatus) & GmacRgmiiIntSts) {
//            printf("%s: GMAC RGMII status is %08x\n", __func__, synopGMACReadReg(GMACdev.MacBase, GmacRgmiiCtrlSts));
            synopGMACReadReg(GMACdev.MacBase, GmacRgmiiCtrlSts);
        }
    }

    interrupt = synopGMAC_get_interrupt_type(&GMACdev);
//    printf("%s:Interrupts to be handled: 0x%08x  %08x\n", __func__, interrupt);

    if (interrupt & synopGMACDmaError) {
        printf("%s::Fatal Bus Error Inetrrupt Seen\n", __func__);
        synopGMAC_disable_dma_tx(&GMACdev);
        synopGMAC_disable_dma_rx(&GMACdev);

        synopGMAC_take_desc_ownership_tx(&GMACdev);
        synopGMAC_take_desc_ownership_rx(&GMACdev);

        synopGMAC_init_tx_rx_desc_queue(&GMACdev);

        synopGMAC_reset(&GMACdev);

        synopGMAC_set_mac_addr(&GMACdev, GmacAddr0High, GmacAddr0Low, &GMACdev.mac_addr[0]);
        synopGMAC_dma_bus_mode_init(&GMACdev, DmaBurstLength32 | DmaDescriptorSkip0/*DmaDescriptorSkip2*/ | DmaDescriptor8Words);
        synopGMAC_dma_control_init(&GMACdev, DmaStoreAndForward | DmaTxSecondFrame | DmaRxThreshCtrl128);
        synopGMAC_init_rx_desc_base(&GMACdev);
        synopGMAC_init_tx_desc_base(&GMACdev);
        synopGMAC_mac_init(&GMACdev);
        synopGMAC_enable_dma_rx(&GMACdev);
        synopGMAC_enable_dma_tx(&GMACdev);

    }

    if ((interrupt & synopGMACDmaRxNormal) ||
        (interrupt & synopGMACDmaRxAbnormal)) {
        if (interrupt & synopGMACDmaRxNormal) {
            LED_GREEN ^= 1;
//            printf("%s:: Rx Normal \n", __func__);
            u32GmacDmaIE &= ~DmaIntRxNormMask;  // disable RX interrupt
        }
        if (interrupt & synopGMACDmaRxAbnormal) {
            LED_RED ^= 1;
//            printf("%s::Abnormal Rx Interrupt Seen %08x\n", __func__, dma_status_reg);
            if (GMACdev.GMAC_Power_down == 0) {
                GMACdev.synopGMACNetStats.rx_over_errors++;
                u32GmacDmaIE &= ~DmaIntRxAbnMask;
//                synopGMAC_resume_dma_rx(&GMACdev);
            }
        }
    }

    if (interrupt & synopGMACDmaRxStopped) {
//        printf("%s::Receiver stopped seeing Rx interrupts\n", __func__); //Receiver gone in to stopped state
        if (GMACdev.GMAC_Power_down == 0) { // If Mac is not in powerdown
            GMACdev.synopGMACNetStats.rx_over_errors++;
            synopGMAC_enable_dma_rx(&GMACdev);
        }
    }

    if (interrupt & synopGMACDmaTxNormal) {
        LED_YELLOW ^= 1;
//        printf("%s::Finished Normal Transmission \n", __func__);
        synop_handle_transmit_over(&GMACdev); // Do whatever you want after the transmission is over
    }

    if (interrupt & synopGMACDmaTxAbnormal) {
        printf("%s::Abnormal Tx Interrupt Seen\n", __func__);
        if (GMACdev.GMAC_Power_down == 0) { // If Mac is not in powerdown
            synop_handle_transmit_over(&GMACdev);
        }
    }

    if (interrupt & synopGMACDmaTxStopped) {
//        printf("%s::Transmitter stopped sending the packets\n", __func__);
        if (GMACdev.GMAC_Power_down == 0) { // If Mac is not in powerdown
            synopGMAC_disable_dma_tx(&GMACdev);
            synopGMAC_take_desc_ownership_tx(&GMACdev);
            synopGMAC_enable_dma_tx(&GMACdev);
//            printf("%s::Transmission Resumed\n", __func__);
        }
    }

    /* Enable the interrrupt before returning from ISR */
    if (interrupt & synopGMACDmaRxNormal) {
#ifdef NX_DRIVER_ENABLE_DEFERRED
      nx_driver_information.nx_driver_information_deferred_events |= NX_DRIVER_DEFERRED_PACKET_RECEIVED;
      _nx_ip_driver_deferred_processing(nx_driver_information.nx_driver_information_ip_ptr);
#else // #ifdef NX_DRIVER_ENABLE_DEFERRED
      if (_nx_driver_hardware_packet_received()) {
        synopGMAC_enable_interrupt(&GMACdev, DmaIntEnable);
      }
#endif  // #ifdef NX_DRIVER_ENABLE_DEFERRED
    }

    synopGMAC_enable_interrupt(&GMACdev, u32GmacDmaIE);
}

static NX_PACKET *nx_net_buffer_alloc(uint32_t n)
{
    UINT status = 0;
    NX_PACKET * packet_ptr = 0;

    if (!nx_driver_information.nx_driver_information_packet_pool_ptr) {
        return NULL;
    }

    status = nx_packet_allocate(nx_driver_information.nx_driver_information_packet_pool_ptr, &packet_ptr, NX_RECEIVE_PACKET, NX_NO_WAIT);
    if (status != NX_SUCCESS) {
        return NULL;
    }

    packet_ptr->nx_packet_prepend_ptr += 2;
    packet_ptr->nx_packet_next = NULL;
    packet_ptr->nx_packet_length = n;
    packet_ptr->nx_packet_append_ptr = packet_ptr->nx_packet_prepend_ptr + n;

    return packet_ptr;
}

static bool _nx_driver_hardware_packet_received()
{
    TX_INTERRUPT_SAVE_AREA;

    TX_DISABLE;

    static uint32_t total_count = 0;
    uint32_t count = 0;
    uint32_t len = 0;
    PKT_FRAME_T* psPktFrame = 0;
    while (1) {	
        len = synop_handle_received_data(&GMACdev, &psPktFrame);
        if (len <= 0) {
          synopGMAC_enable_interrupt(&GMACdev, DmaIntEnable);
          break;
        }
        NX_PACKET *packet_ptr = nx_net_buffer_alloc(len);
        if (packet_ptr) {
            count ++; 
            total_count ++;
//            printf("_nx_driver_hardware_packet_received len:%08x(%d)\n", len, len);
            memcpy(packet_ptr->nx_packet_prepend_ptr, (void *)psPktFrame, len);
            _nx_driver_transfer_to_netx(nx_driver_information.nx_driver_information_ip_ptr, packet_ptr);
        } else {
            printf("_nx_driver_hardware_packet_received: Dropped to floor!\n");
        }
        synopGMAC_set_rx_qptr(&GMACdev, (u32)psPktFrame, PKT_FRAME_BUF_SIZE, 0);
    }

    TX_RESTORE;

//    printf("_nx_driver_hardware_packet_received total %d local %d\n", total_count, count);

    return false;
}

#if 0
static void _nx_hex_dump(char *desc, void *addr, int len) {
  int i;
  unsigned char buff[17];
  unsigned char *pc = (unsigned char*)addr;

  // Output description if given.
  if (desc != NULL)
    printf ("%s:\n", desc);

  // Process every byte in the data.
  for (i = 0; i < len; i++) {
    // Multiple of 16 means new line (with line offset).
    if ((i % 16) == 0) {
      // Just don't print ASCII for the zeroth line.
      if (i != 0)
        printf ("  %s\n", buff);

      // Output the offset.
      printf ("  %04x ", i);
    }

    // Now the hex code for the specific character.
    printf (" %02x", pc[i]);

    // And store a printable ASCII character for later.
    if ((pc[i] < 0x20) || (pc[i] > 0x7e))
      buff[i % 16] = '.';
    else
      buff[i % 16] = pc[i];
    buff[(i % 16) + 1] = '\0';
  }

  // Pad out last line if not exactly 16 characters.
  while ((i % 16) != 0) {
    printf ("   ");
    i++;
  }

  // And print the final ASCII bit.
  printf ("  %s\n", buff);
}
#endif  // #if 0
