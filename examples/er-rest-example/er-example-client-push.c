/*
 * Copyright (c) 2013, Institute for Pervasive Computing, ETH Zurich
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 */

/**
 * \file
 *      Erbium (Er) CoAP client example.
 * \author
 *      Matthias Kovatsch <kovatsch@inf.ethz.ch>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "contiki.h"
#include "contiki-net.h"
#include "er-coap-engine.h"
#include "dev/button-sensor.h"
#include "dev/leds.h"

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#define PRINT6ADDR(addr) PRINTF("[%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x]", ((uint8_t *)addr)[0], ((uint8_t *)addr)[1], ((uint8_t *)addr)[2], ((uint8_t *)addr)[3], ((uint8_t *)addr)[4], ((uint8_t *)addr)[5], ((uint8_t *)addr)[6], ((uint8_t *)addr)[7], ((uint8_t *)addr)[8], ((uint8_t *)addr)[9], ((uint8_t *)addr)[10], ((uint8_t *)addr)[11], ((uint8_t *)addr)[12], ((uint8_t *)addr)[13], ((uint8_t *)addr)[14], ((uint8_t *)addr)[15])
#define PRINTLLADDR(lladdr) PRINTF("[%02x:%02x:%02x:%02x:%02x:%02x]", (lladdr)->addr[0], (lladdr)->addr[1], (lladdr)->addr[2], (lladdr)->addr[3], (lladdr)->addr[4], (lladdr)->addr[5])
#else
#define PRINTF(...)
#define PRINT6ADDR(addr)
#define PRINTLLADDR(addr)
#endif

/* for handling serial-line events: */
#include "dev/serial-line.h"
static uint8_t set_etimer = 0;
static uint8_t pause_coap = 1;
unsigned long clock_start; // energest
unsigned long cpu_start_time; // energest
unsigned long lpm_start_time; // energest
unsigned long tx_start_time; // energest
unsigned long rx_start_time; // energest
unsigned long irq_start_time; // energest
uint32_t packet_transmitted_start;
uint32_t packets_received_start;

/*---------------------------------------------------------------------------*/
/* Packet sniffer */
#include "rime.h"
static uint32_t packets_received = 0;
static uint32_t packets_transmitted = 0;

void packet_received(void) { packets_received++; }
void packet_transmitted(int mac_status) { packets_transmitted++; }

RIME_SNIFFER(packet_counter, &packet_received, &packet_transmitted);
/*---------------------------------------------------------------------------*/

/* FIXME: This server address is hard-coded for Cooja and link-local for unconnected border router. */
// #define SERVER_NODE(ipaddr)   uip_ip6addr(ipaddr, 0xfe80, 0, 0, 0, 0x0212, 0x7402, 0x0002, 0x0202)      /* cooja2 */
/* #define SERVER_NODE(ipaddr)   uip_ip6addr(ipaddr, 0xbbbb, 0, 0, 0, 0, 0, 0, 0x1) */
// coap://[2001:6a8:1d80:23::157]:5683/ms/0/s/t
#define SERVER_NODE(ipaddr)   uip_ip6addr(ipaddr, 0x2001, 0x6a8, 0x1d80, 0x23, 0, 0, 0, 0x157) // rd.comacod.test.iminds.be

#define LOCAL_PORT      UIP_HTONS(COAP_DEFAULT_PORT + 1)
#define REMOTE_PORT     UIP_HTONS(COAP_DEFAULT_PORT)

#define TOGGLE_INTERVAL 30

PROCESS(er_example_client, "Erbium Example Client");
PROCESS(serial_comms, "Energest over serialline");
AUTOSTART_PROCESSES(&er_example_client, &serial_comms);

uip_ipaddr_t server_ipaddr;
static struct etimer et;

static coap_packet_t request[1];      /* This way the packet can be treated as pointer as usual. */
static int32_t count = 0;

/* Example URIs that can be queried. */
#define NUMBER_OF_URLS 5
/* leading and ending slashes only for demo purposes, get cropped automatically when setting the Uri-Path */
char *service_urls[NUMBER_OF_URLS] =
{ ".well-known/core", "/actuators/toggle", "battery/", "error/in//path", "ms/0/s/t" };
#if PLATFORM_HAS_BUTTON
static int uri_switch = 0;
#endif

/* This function is will be passed to COAP_BLOCKING_REQUEST() to handle responses. */
void
client_chunk_handler(void *response)
{
#if DEBUG
  const uint8_t *chunk;

  int len = coap_get_payload(response, &chunk);

  PRINTF("|%.*s", len, (char *)chunk);
#endif
  PRINTF("client_chunk_handler() \n");
  leds_toggle(LEDS_GREEN);
}

PROCESS_THREAD(er_example_client, ev, data)
{
  PROCESS_BEGIN();

  SERVER_NODE(&server_ipaddr);

  /* receives all CoAP messages */
  coap_init_engine();

  etimer_set(&et, TOGGLE_INTERVAL * CLOCK_SECOND);
  count = 0;

#if PLATFORM_HAS_BUTTON
  SENSORS_ACTIVATE(button_sensor);
  printf("Press a button to request %s\n", service_urls[uri_switch]);
#endif

  while(1) {
    PROCESS_YIELD();

    if(etimer_expired(&et)) {
      if (!pause_coap) {
        //cc2420_on();
        NETSTACK_MAC.on();

        leds_toggle(LEDS_GREEN);

        PRINTF("--PUT request timer--\n");

        /* prepare request, TID is set by COAP_BLOCKING_REQUEST() */
        // Send CON update every 20 messages:
        coap_message_type_t message_type = COAP_TYPE_CON;
        // sending NON messages as a client does not work well with the BLOCKING_REQUEST macro
        // if (count % 20 == 0) {
        //   message_type = COAP_TYPE_CON;
        // }
        coap_init_message(request, message_type, COAP_PUT, 0);
        coap_set_header_uri_path(request, service_urls[4]);

        //const char msg[] = "Toggle!";
        char msg[8] = {0};
        snprintf(msg, 8, "20.%02luC", (count % 100));

        coap_set_payload(request, (uint8_t *)msg, sizeof(msg) - 1);

        PRINT6ADDR(&server_ipaddr);
        PRINTF(" : %u\n", UIP_HTONS(REMOTE_PORT));


        COAP_BLOCKING_REQUEST(&server_ipaddr, REMOTE_PORT, request, client_chunk_handler);
        //cc2420_off();
        NETSTACK_MAC.off(0);

        PRINTF("\n--Done--\n");


        count++;
      }

      if (set_etimer) {
        set_etimer = 0;
        etimer_set(&et, TOGGLE_INTERVAL * CLOCK_SECOND);
        PRINTF("etimer set\n");
      } else {
        etimer_reset(&et);
        PRINTF("etimer reset\n");
      }
    }
  }

PROCESS_END();
}

PROCESS_THREAD(serial_comms, ev, data)
{
  PROCESS_BEGIN();

  /* initialize serial line */
  uart0_set_input(serial_line_input_byte);
  serial_line_init();

	/* Rime sniffer */
  rime_sniffer_add(&packet_counter);

  while(1) {
    PROCESS_WAIT_EVENT();
     if(ev == serial_line_event_message) {
      char *line = (char *)data;
      if (line[0] == '?' && line[1] == 'E') { // request to print energest values:
        printf("%lu %lu %lu %lu %lu %lu %lu %lu\n",
            clock_time() - clock_start,
            energest_type_time(ENERGEST_TYPE_CPU) - cpu_start_time,
            energest_type_time(ENERGEST_TYPE_LPM) - lpm_start_time ,
            energest_type_time(ENERGEST_TYPE_TRANSMIT) - tx_start_time,
            energest_type_time(ENERGEST_TYPE_LISTEN) - rx_start_time,
            energest_type_time(ENERGEST_TYPE_IRQ) - irq_start_time,
            packets_transmitted - packet_transmitted_start,
            packets_received - packets_received_start);

      } else if(line[0] == '!' && line[1] == 'S') { // start/stop coap transmitting
        if(!pause_coap) {
          pause_coap = 1;
        } else {
          pause_coap = 0;

          // If this is the first time that we will start sending coap requests, then set the energest start and rime sniffer start values
          if (rx_start_time == 0) {
            clock_start = clock_time();
            rx_start_time = energest_type_time(ENERGEST_TYPE_LISTEN);
            lpm_start_time = energest_type_time(ENERGEST_TYPE_LPM);
            cpu_start_time = energest_type_time(ENERGEST_TYPE_CPU);
            tx_start_time = energest_type_time(ENERGEST_TYPE_TRANSMIT);
            irq_start_time = energest_type_time(ENERGEST_TYPE_IRQ);
            packet_transmitted_start = packets_transmitted;
            packets_received_start  = packets_received;
          }

          // Set etimer to expire now
          set_etimer = 1;
          etimer_set(&et, 0);
        }

        printf("pause_coap = %d\n", pause_coap);
      }
    }
  }

PROCESS_END();
}
