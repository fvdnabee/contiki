#include "contiki.h"
#include <stdio.h>
#include <string.h>

#include "dev/cc2520/cc2520.h"
#include "dev/leds.h"
#include "dev/serial-line.h"
#include "dev/slip.h"
#include "dev/watchdog.h"
#include "net/netstack.h"
#include "net/mac/frame802154.h"

#ifdef UART_DEBUG
#warning Using secondary UART port
#include "dev/uart0.h"
#define UART_INIT uart0_init
#define UART_SET_INPUT uart0_set_input
#define UART_ACTIVE uart0_active
#else
#include "dev/uart1.h"
#define UART_INIT uart1_init
#define UART_SET_INPUT uart1_set_input
#define UART_ACTIVE uart1_active
#endif

#ifndef ENABLE_PRINTF
#define printf(...)
#endif

#if WITH_UIP6
#include "net/ipv6/uip-ds6.h"
#endif /* WITH_UIP6 */

#include "net/rime/rime.h"

#include "sys/node-id.h"
#include "sys/autostart.h"

#if UIP_CONF_ROUTER

#ifndef UIP_ROUTER_MODULE
#ifdef UIP_CONF_ROUTER_MODULE
#define UIP_ROUTER_MODULE UIP_CONF_ROUTER_MODULE
#else /* UIP_CONF_ROUTER_MODULE */
#define UIP_ROUTER_MODULE rimeroute
#endif /* UIP_CONF_ROUTER_MODULE */
#endif /* UIP_ROUTER_MODULE */

extern const struct uip_router UIP_ROUTER_MODULE;
#endif /* UIP_CONF_ROUTER */

#ifndef WITH_UIP
#define WITH_UIP 0
#endif

unsigned short node_id;

#if WITH_UIP
#include "net/uip.h"
#include "net/uip-fw.h"
#include "net/uip-fw-drv.h"
#include "net/uip-over-mesh.h"
static struct uip_fw_netif slipif =
{UIP_FW_NETIF(192,168,1,2, 255,255,255,255, slip_send)};
static struct uip_fw_netif meshif =
{UIP_FW_NETIF(172,16,0,0, 255,255,0,0, uip_over_mesh_send)};

#endif /* WITH_UIP */

#define UIP_OVER_MESH_CHANNEL 8
#if WITH_UIP
static uint8_t is_gateway;
#endif /* WITH_UIP */

void init_platform(void);

/*---------------------------------------------------------------------------*/
void uip_log(char *msg) { puts(msg); }
/*---------------------------------------------------------------------------*/
#ifndef RF_CHANNEL
#define RF_CHANNEL              26
#endif

#ifndef NODE_ID
#warning No node ID specified, using default 0x02
#define NODE_ID	0x02
#else
#warning Using predefined node ID
#endif /* NODE_ID */
static void
set_rime_addr(void)
{
	linkaddr_t n_addr;
	int i;

	memset(&n_addr, 0, sizeof(linkaddr_t));

	//	Set node address
#if UIP_CONF_IPV6
	//memcpy(addr.u8, ds2411_id, sizeof(addr.u8));
	n_addr.u8[7] = node_id & 0xff;
	n_addr.u8[6] = node_id >> 8;
#else
	/* if(node_id == 0) {
	for(i = 0; i < sizeof(linkaddr_t); ++i) {
	  addr.u8[i] = ds2411_id[7 - i];
	}
  } else {
	addr.u8[0] = node_id & 0xff;
	addr.u8[1] = node_id >> 8;
  }*/
	n_addr.u8[0] = node_id & 0xff;
	n_addr.u8[1] = node_id >> 8;
#endif

	linkaddr_set_node_addr(&n_addr);
	printf("Rime started with address ");
	for(i = 0; i < sizeof(n_addr.u8) - 1; i++) {
		printf("%d.", n_addr.u8[i]);
	}
	printf("%d\n", n_addr.u8[i]);
}
/*---------------------------------------------------------------------------*/
#if WITH_UIP
static void
set_gateway(void)
{
	if(!is_gateway) {
		leds_on(LEDS_RED);
		//printf("%d.%d: making myself the IP network gateway.\n\n",
		//   linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1]);
		//printf("IPv4 address of the gateway: %d.%d.%d.%d\n\n",
		//  uip_ipaddr_to_quad(&uip_hostaddr));
		uip_over_mesh_set_gateway(&linkaddr_node_addr);
		uip_over_mesh_make_announced_gateway();
		is_gateway = 1;
	}
}
#endif /* WITH_UIP */
/*---------------------------------------------------------------------------*/
int
main(int argc, char **argv){

	msp430_cpu_init();
	clock_init();
	leds_init();

	clock_wait(2);

	UART_INIT(115200);

	putchar('\n'); /* Force include putchar */

#if WITH_UIP
	slip_arch_init(115200);
#endif /* WITH_UIP */

	clock_wait(1);

	rtimer_init();

	node_id = NODE_ID;

	process_init();
	process_start(&etimer_process, NULL);

	ctimer_init();

	init_platform();

	set_rime_addr();

	__dint();
	NETSTACK_CONF_RADIO.init();
	__eint();

	{
		uint8_t longaddr[8];
		uint16_t shortaddr;

		shortaddr = (linkaddr_node_addr.u8[0] << 8) + linkaddr_node_addr.u8[1];
		memset(longaddr, 0, sizeof(longaddr));
		linkaddr_copy((linkaddr_t *)&longaddr, &linkaddr_node_addr);

		printf("MAC %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x ",
			   longaddr[0], longaddr[1], longaddr[2], longaddr[3],
				longaddr[4], longaddr[5], longaddr[6], longaddr[7]);

		cc2520_set_pan_addr(IEEE802154_PANID, shortaddr, longaddr);

	}
	cc2520_set_channel(RF_CHANNEL);

	printf(CONTIKI_VERSION_STRING " started. ");
	if(node_id > 0) {
		printf("Node id is set to %u.\n", node_id);
	} else {
		printf("Node id is not set.\n");
	}

#if WITH_UIP6
	/* memcpy(&uip_lladdr.addr, ds2411_id, sizeof(uip_lladdr.addr)); */
	memcpy(&uip_lladdr.addr, linkaddr_node_addr.u8,
		   UIP_LLADDR_LEN > LINKADDR_SIZE ? LINKADDR_SIZE : UIP_LLADDR_LEN);

	/* Initialize network stack */
	queuebuf_init();
	NETSTACK_RDC.init();
	NETSTACK_MAC.init();
	NETSTACK_NETWORK.init();

	printf("%s %s, channel check rate %lu Hz, radio channel %u\n",
		   NETSTACK_MAC.name, NETSTACK_RDC.name,
		   CLOCK_SECOND / (NETSTACK_RDC.channel_check_interval() == 0 ? 1:
																		NETSTACK_RDC.channel_check_interval()),
		   RF_CHANNEL);

	process_start(&tcpip_process, NULL);

	printf("Tentative link-local IPv6 address ");
	{
		uip_ds6_addr_t *lladdr;
		int i;
		lladdr = uip_ds6_get_link_local(-1);
		for(i = 0; i < 7; ++i) {
			printf("%02x%02x:", lladdr->ipaddr.u8[i * 2],
					lladdr->ipaddr.u8[i * 2 + 1]);
		}
		printf("%02x%02x\n", lladdr->ipaddr.u8[14], lladdr->ipaddr.u8[15]);
	}

	if(!UIP_CONF_IPV6_RPL) {
		uip_ipaddr_t ipaddr;
		int i;
		uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
		uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
		uip_ds6_addr_add(&ipaddr, 0, ADDR_TENTATIVE);
		printf("Tentative global IPv6 address ");
		for(i = 0; i < 7; ++i) {
			printf("%02x%02x:",
				   ipaddr.u8[i * 2], ipaddr.u8[i * 2 + 1]);
		}
		printf("%02x%02x\n",
			   ipaddr.u8[7 * 2], ipaddr.u8[7 * 2 + 1]);
	}

#else /* WITH_UIP6 */

	NETSTACK_RDC.init();
	NETSTACK_MAC.init();
	NETSTACK_NETWORK.init();

	printf("%s %s, channel check rate %lu Hz, radio channel %u\n",
		   NETSTACK_MAC.name, NETSTACK_RDC.name,
		   CLOCK_SECOND / (NETSTACK_RDC.channel_check_interval() == 0? 1:
																	   NETSTACK_RDC.channel_check_interval()),
		   RF_CHANNEL);
#endif /* WITH_UIP6 */

#if !WITH_UIP && !WITH_UIP6
	UART_SET_INPUT(serial_line_input_byte);
	serial_line_init();
#endif

#if TIMESYNCH_CONF_ENABLED
	timesynch_init();
	timesynch_set_authority_level((linkaddr_node_addr.u8[0] << 4) + 16);
#endif /* TIMESYNCH_CONF_ENABLED */

#if WITH_UIP
	process_start(&tcpip_process, NULL);
	process_start(&uip_fw_process, NULL);	/* Start IP output */
	process_start(&slip_process, NULL);

	slip_set_input_callback(set_gateway);

	{
		uip_ipaddr_t hostaddr, netmask;

		uip_init();

		uip_ipaddr(&hostaddr, 172,16,
				   linkaddr_node_addr.u8[0],linkaddr_node_addr.u8[1]);
		uip_ipaddr(&netmask, 255,255,0,0);
		uip_ipaddr_copy(&meshif.ipaddr, &hostaddr);

		uip_sethostaddr(&hostaddr);
		uip_setnetmask(&netmask);
		uip_over_mesh_set_net(&hostaddr, &netmask);
		/*    uip_fw_register(&slipif);*/
		uip_over_mesh_set_gateway_netif(&slipif);
		uip_fw_default(&meshif);
		uip_over_mesh_init(UIP_OVER_MESH_CHANNEL);
		printf("uIP started with IP address %d.%d.%d.%d\n",
			   uip_ipaddr_to_quad(&hostaddr));
	}
#endif /* WITH_UIP */

	watchdog_stop();

	autostart_start(autostart_processes);

  energest_init();
  ENERGEST_ON(ENERGEST_TYPE_CPU);

	while(1) {

		while(process_run() > 0);

		__dint();

		if(process_nevents() != 0 || UART_ACTIVE())
			__eint();
		else {
      static unsigned long irq_energest = 0;
      ENERGEST_OFF(ENERGEST_TYPE_CPU);
      ENERGEST_ON(ENERGEST_TYPE_LPM);
      /* We only want to measure the processing done in IRQs when we
	 are asleep, so we discard the processing time done when we
	 were awake. */
      energest_type_set(ENERGEST_TYPE_IRQ, irq_energest);

			_BIS_SR(GIE | SCG0 | SCG1 | CPUOFF);
      /* We get the current processing time for interrupts that was
	 done during the LPM and store it for next time around.  */
      dint();
      irq_energest = energest_type_time(ENERGEST_TYPE_IRQ);
      eint();
      ENERGEST_OFF(ENERGEST_TYPE_LPM);
      ENERGEST_ON(ENERGEST_TYPE_CPU);

    }

	}
}
/*---------------------------------------------------------------------------*/
#if LOG_CONF_ENABLED
void
log_message(char *m1, char *m2)
{
	printf("%s%s\n", m1, m2);
}
#endif /* LOG_CONF_ENABLED */
