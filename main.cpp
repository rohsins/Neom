#include <LPC17xx.h>
#include <UART_LPC17xx.h>
#include <cmsis_os.h>
#include "ITM_ARM.h"
#include <string.h>
#include <Driver_ETH.h>
#include <Driver_ETH_MAC.h>
#include <Driver_ETH_PHY.h>
#include "rl_net.h"

#define IER_RBR 1U << 0
#define IER_THRE 1U << 1
#define IER_RLS 1U << 2

extern ARM_DRIVER_USART Driver_USART0;
extern ARM_DRIVER_USART Driver_USART1;
extern ARM_DRIVER_ETH_MAC Driver_ETH_MAC0;
extern ARM_DRIVER_ETH_PHY Driver_ETH_PHY0;

ARM_USART_STATUS Driver_USART1_STATUS;
USART_TRANSFER_INFO Driver_USART1_INFO;

static ARM_ETH_MAC_CAPABILITIES capabilities;
static ARM_DRIVER_ETH_MAC *mac;
static ARM_DRIVER_ETH_PHY *phy;
static ARM_ETH_MAC_ADDR macAddress;

const char mac_addr[]  = { "AE-30-6C-A2-45-5A" };
const char ip_addr[]   = { "192.168.5.217" };
const char def_gw[]    = { "192.168.5.1" };
const char net_mask[]  = { "255.255.255.0" };
const char pri_dns[]   = { "8.8.8.8" };
const char sec_dns[]   = { "8.8.4.4" };
const char host_name[] = { "roh" };
bool DHCP_enabled      = false;

bool ethernetDataFlag = false;
bool triacToggle = false;

int32_t tcp_sock;
char tempwhat[32];

void USART_callback(uint32_t event)
{
//		static int i;
	
    switch (event)
    {
    case ARM_USART_EVENT_RECEIVE_COMPLETE: 
//				itmPrintln("receive complete");
//					ringBufferWrite(pData);
//					ringBufferRead(readout);
//					itmPrint("readout:"); itmPrintlnInt(*readout);
//					i++;
//				itmPrintln(temp);
				break;
    case ARM_USART_EVENT_TRANSFER_COMPLETE:
//			itmPrintln("transfer complete");
			break;
    case ARM_USART_EVENT_SEND_COMPLETE:
//			itmPrintln("send complete");
			break;
    case ARM_USART_EVENT_TX_COMPLETE:
//			itmPrintln("tx complete");
        /* Success: Wakeup Thread */
//        osSignalSet(tid_myUART_Thread, 0x01);
        break;
 
    case ARM_USART_EVENT_RX_TIMEOUT:
//			itmPrintln("rx timeout");
         __breakpoint(0);  /* Error: Call debugger or replace with custom error handling */
        break;
 
    case ARM_USART_EVENT_RX_OVERFLOW:
//			itmPrintln("rx overflow");
			break;
    case ARM_USART_EVENT_TX_UNDERFLOW:
//        __breakpoint(0);  /* Error: Call debugger or replace with custom error handling */
		break;		
	}
}

void uart0Initialize(void) {
	Driver_USART0.Initialize(USART_callback);
	Driver_USART0.PowerControl(ARM_POWER_FULL);
	Driver_USART0.Control(ARM_USART_MODE_ASYNCHRONOUS| ARM_USART_DATA_BITS_8| ARM_USART_PARITY_NONE | ARM_USART_STOP_BITS_1 | ARM_USART_FLOW_CONTROL_NONE,115200);
	Driver_USART0.Control(ARM_USART_CONTROL_TX,1);
	Driver_USART0.Control(ARM_USART_CONTROL_RX,1);      

	NVIC_EnableIRQ(UART0_IRQn);
	LPC_UART0->IER = IER_RBR | IER_THRE |IER_RLS; //Enable Interrupt
}

void uart0UnInitialize(void) {
	Driver_USART0.Uninitialize();
}

void uart1Initialize(void) {
	Driver_USART1.Initialize(USART_callback);
	Driver_USART1.PowerControl(ARM_POWER_FULL);
	Driver_USART1.Control(ARM_USART_MODE_ASYNCHRONOUS| ARM_USART_DATA_BITS_8| ARM_USART_PARITY_NONE | ARM_USART_STOP_BITS_1 | ARM_USART_FLOW_CONTROL_NONE,115200);
	Driver_USART1.Control(ARM_USART_CONTROL_TX,1);
	Driver_USART1.Control(ARM_USART_CONTROL_RX,1);  

	NVIC_EnableIRQ(UART1_IRQn);
	LPC_UART1->IER |= IER_RBR | IER_THRE |IER_RLS; //Enable Interrupt
}

void uart1UnInitialize(void) {
	Driver_USART1.Uninitialize();
}

void ethernetEvent(uint32_t event) {
//	itmPrintln("we are inside ethernetEvent");
}

void ethernetInitialize(void) {
//	mac = &Driver_ETH_MAC0;
//  phy = &Driver_ETH_PHY0;
//	capabilities = mac->GetCapabilities();
//	mac->PowerControl(ARM_POWER_FULL);
//	mac->GetMacAddress(macAddress);
//	mac->SetMacAddress(macAddress); 
//	mac->SetAddressFilter(macAddress, 0);
//	mac->Control(ARM_ETH_MAC_CONFIGURE, ARM_ETH_MAC_SPEED_100M | ARM_ETH_MAC_DUPLEX_FULL);
//	mac->Initialize(ethernetEvent);
	
	mac = &Driver_ETH_MAC0;
  phy = &Driver_ETH_PHY0;
 
  capabilities = mac->GetCapabilities ();
   
  mac->Initialize (ethernetEvent);
  mac->PowerControl (ARM_POWER_FULL);
 
  if (capabilities.mac_address == 1)  {
    mac->SetMacAddress(&macAddress);
  }
  else {
    mac->GetMacAddress(&macAddress);
  }
 
  if (phy->Initialize (mac->PHY_Read, mac->PHY_Write) == ARM_DRIVER_OK) {
    phy->PowerControl (ARM_POWER_FULL);
    phy->SetInterface (ARM_ETH_INTERFACE_RMII);
    phy->SetMode (ARM_ETH_PHY_AUTO_NEGOTIATE);
  }
	
	ARM_ETH_LINK_INFO info = phy->GetLinkInfo ();
    mac->Control(ARM_ETH_MAC_CONFIGURE,
                 info.speed  << ARM_ETH_MAC_SPEED_Pos  |
                 info.duplex << ARM_ETH_MAC_DUPLEX_Pos |
                 ARM_ETH_MAC_ADDRESS_BROADCAST);
    mac->Control(ARM_ETH_MAC_CONTROL_TX, 1);
    mac->Control(ARM_ETH_MAC_CONTROL_RX, 1);
	
}

static ARM_ETH_LINK_STATE ethernet_link;  
 
void ethernet_check_link_status (void) {
  ARM_ETH_LINK_STATE link;
 
  link = phy->GetLinkState ();
  if (link == ethernet_link) {    
    return;                              
  }
                                       
  ethernet_link = link;   
  if (link == ARM_ETH_LINK_UP) {     
    ARM_ETH_LINK_INFO info = phy->GetLinkInfo ();
    mac->Control(ARM_ETH_MAC_CONFIGURE,
                 info.speed  << ARM_ETH_MAC_SPEED_Pos  |
                 info.duplex << ARM_ETH_MAC_DUPLEX_Pos |
                 ARM_ETH_MAC_ADDRESS_BROADCAST);
    mac->Control(ARM_ETH_MAC_CONTROL_TX, 1);
    mac->Control(ARM_ETH_MAC_CONTROL_RX, 1);
  }
  else {                     
    mac->Control(ARM_ETH_MAC_FLUSH, ARM_ETH_MAC_FLUSH_TX | ARM_ETH_MAC_FLUSH_RX);
    mac->Control(ARM_ETH_MAC_CONTROL_TX, 0);
    mac->Control(ARM_ETH_MAC_CONTROL_RX, 0);
  }
}

void ethernetConfig(void) {
	uint8_t buf[8]; 
 
  netSYS_SetHostName (host_name);
 
  netMAC_aton (mac_addr, buf);
  netIF_SetOption (NET_IF_CLASS_ETH | 0, netIF_OptionMAC_Address, buf, NET_ADDR_ETH_LEN);
 
  if (DHCP_enabled == false) {
    netDHCP_Disable (0);
 
    netIP_aton (ip_addr, NET_ADDR_IP4, buf);
    netIF_SetOption (NET_IF_CLASS_ETH | 0, netIF_OptionIP4_Address, buf, NET_ADDR_IP4_LEN);
 
    netIP_aton (net_mask, NET_ADDR_IP4, buf);
    netIF_SetOption (NET_IF_CLASS_ETH | 0, netIF_OptionIP4_SubnetMask, buf, NET_ADDR_IP4_LEN);
 
    netIP_aton (def_gw, NET_ADDR_IP4, buf);
    netIF_SetOption (NET_IF_CLASS_ETH | 0, netIF_OptionIP4_DefaultGateway, buf, NET_ADDR_IP4_LEN);
 
    netIP_aton (pri_dns, NET_ADDR_IP4, buf);
    netIF_SetOption (NET_IF_CLASS_ETH | 0, netIF_OptionIP4_PrimaryDNS, buf, NET_ADDR_IP4_LEN);
 
    netIP_aton (sec_dns, NET_ADDR_IP4, buf);
    netIF_SetOption (NET_IF_CLASS_ETH | 0, netIF_OptionIP4_SecondaryDNS, buf, NET_ADDR_IP4_LEN);
  }
}

void ledInitialize(void) {
	LPC_PINCON->PINSEL1 = 0x00;
	LPC_PINCON->PINMODE1 = 0x00;
	LPC_PINCON->PINMODE_OD1 = 0x00;
	LPC_GPIO1->FIODIR2 = 0xFF;
	LPC_GPIO1->FIOSET2 = 0XFF;
	
	/* for triac gpio control */
	LPC_PINCON->PINSEL0 = 0x00;
	LPC_PINCON->PINMODE1 = 0x00;
	LPC_PINCON->PINMODE_OD0 = 0x00;
	LPC_GPIO0->FIODIR1 = 0x04;
	LPC_GPIO0->FIOSET1 = 0X04;
}                                                                                                                                      

uint32_t tcp_cb_func (int32_t socket, netTCP_Event event, const NET_ADDR *addr, const uint8_t *buf, uint32_t len) {
  switch (event) {
    case netTCP_EventConnect:
      if (addr->addr_type == NET_ADDR_IP4) {
        if (addr->addr[0] == 192  &&
            addr->addr[1] == 168  &&
            addr->addr[2] == 5    &&
            addr->addr[3] == 117) {
							ethernetDataFlag = true;
//						itmPrintln("received from mobile");
          return (1);
        }
      }
      return (0);
 
    case netTCP_EventEstablished:
      // Connection established
      break;
 
    case netTCP_EventClosed:
      // Connection was properly closed
      break;
 
    case netTCP_EventAborted:
      // Connection is for some reason aborted
      break;
 
    case netTCP_EventACK:
      // Previously sent data acknowledged
      break;
 
    case netTCP_EventData:
			int i = 0;
			memset(tempwhat, 0, sizeof(tempwhat));
			while (i < len) {
				tempwhat[i] = buf[i];
				i++;
			}
      break;
  }
  return (0);
}

void socketListen(void) {
	tcp_sock = netTCP_GetSocket (tcp_cb_func);
	if (tcp_sock >= 0) {
		netTCP_Listen (tcp_sock, 60200);	
		netTCP_SetOption (tcp_sock, netTCP_OptionTimeout, 30);
		netTCP_SetOption (tcp_sock, netTCP_OptionKeepAlive, 1);
		netTCP_SetOption (tcp_sock, netTCP_OptionDelayedACK, 1);
	}
}

void heartBeatThread(void const *arg) {
	while (1) {
		LPC_GPIO1->FIOCLR2 = 0XFF;
		osDelay(70);
		LPC_GPIO1->FIOSET2 = 0XFF;
		osDelay(1000);
		if (ethernetDataFlag == true) {
			ethernetDataFlag = false;
			char checkstring[32]={'\0','\x06','A','l','t','i','u','m','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0'};
	//			char checkstring1[32]={'\0','\x06','A','l','t','i','u','m','\0'};
			if (strcmp(tempwhat, checkstring) == 0) {
				if (triacToggle == false) {
					LPC_GPIO0->FIOCLR1 = 0X04;
					triacToggle = true;
					int s = 10;
					while (s > 1) {
						LPC_GPIO1->FIOCLR2 = 0XFF;
						osDelay(100);
						LPC_GPIO1->FIOSET2 = 0XFF;
						osDelay(100);
						s--;
					}
				} else { 
					LPC_GPIO0->FIOSET1 = 0X04;
					triacToggle = false;
				}
			}
		}
	}
}
osThreadDef(heartBeatThread, osPriorityNormal, 1, 0);

int main(void) {
	
	SystemCoreClockUpdate ();
	SysTick_Config(SystemCoreClock/1000);
		
	osKernelInitialize();
	osDelay(100);
	netInitialize();
	osDelay(100);
	ethernetInitialize();
//	ethernetConfig();
	osDelay(100);
	ledInitialize();
	
	osThreadCreate(osThread(heartBeatThread), NULL);
	socketListen();
	
	osKernelStart();
	
	return 0;
}
