
/*
 * Auto generated Run-Time-Environment Component Configuration File
 *      *** Do not modify ! ***
 *
 * Project: 'Neom' 
 * Target:  'Release' 
 */

#ifndef RTE_COMPONENTS_H
#define RTE_COMPONENTS_H


/*
 * Define the Device Header File: 
 */
#define CMSIS_device_header "LPC17xx.h"

#define RTE_CMSIS_RTOS                  /* CMSIS-RTOS */
        #define RTE_CMSIS_RTOS_RTX              /* CMSIS-RTOS Keil RTX */
#define RTE_DEVICE_STARTUP_LPC17XX      /* Device Startup for NXP17XX */
#define RTE_Drivers_ETH_MAC0            /* Driver ETH_MAC0 */
#define RTE_Drivers_PHY_LAN8720         /* Driver PHY LAN8720 */
#define RTE_Drivers_USART0              /* Driver USART0 */
        #define RTE_Drivers_USART1              /* Driver USART1 */
        #define RTE_Drivers_USART2              /* Driver USART2 */
        #define RTE_Drivers_USART3              /* Driver USART3 */
#define RTE_Network_Core                /* Network Core */
          #define RTE_Network_IPv4                /* Network IPv4 Stack */
          #define RTE_Network_Release             /* Network Release Version */
#define RTE_Network_Interface_ETH_0     /* Network Interface ETH 0 */
#define RTE_Network_Legacy              /* Network Legacy */
#define RTE_Network_Socket_BSD          /* Network Socket BSD */
#define RTE_Network_Socket_TCP          /* Network Socket TCP */
#define RTE_Network_Socket_UDP          /* Network Socket UDP */

#endif /* RTE_COMPONENTS_H */
