/*
 * Copyright (c) 2012, Texas Instruments Incorporated - http://www.ti.com/
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
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * \addtogroup cc2538
 * @{
 *
 * \defgroup cc2538-examples cc2538dk Example Projects
 * @{
 *
 * \defgroup cc2538-demo cc2538dk Demo Project
 *
 *   Example project demonstrating the cc2538dk functionality
 *
 *   This assumes that you are using a SmartRF06EB with a cc2538 EM
 *
 * - Boot sequence: LEDs flashing, LED2 followed by LED3 then LED4
 * - etimer/clock : Every LOOP_INTERVAL clock ticks the LED defined as
 *                  LEDS_PERIODIC will turn on
 * - rtimer       : Exactly LEDS_OFF_HYSTERISIS rtimer ticks later,
 *                  LEDS_PERIODIC will turn back off
 * - Buttons      :
 *                - BTN_DOWN turns on LEDS_REBOOT and causes a watchdog reboot
 *                - BTN_UP to soft reset (SYS_CTRL_PWRDBG::FORCE_WARM_RESET)
 *                - BTN_LEFT and BTN_RIGHT flash the LED defined as LEDS_BUTTON
 * - UART         : Every LOOP_INTERVAL the EM will print something over the
 *                  UART. Receiving an entire line of text over UART (ending
 *                  in \\r) will cause LEDS_SERIAL_IN to toggle
 * - Radio comms  : BTN_SELECT sends a rime broadcast. Reception of a rime
 *                  packet will toggle LEDs defined as LEDS_RF_RX
 *
 * @{
 *
 * \file
 *     Example demonstrating the cc2538dk platform
 */

#include "contiki.h"
//#include "cpu.h"
//#include "sys/etimer.h"
//#include "sys/rtimer.h"
#include "dev/leds.h"
#include "dev/cc2520/cc2520.h"
#include "dev/watchdog.h"
#include "dev/serial-line.h"
#include "dev/hwconf.h"
#include "sys/compower.h"
#include "rime.h"

#include <stdio.h>
#include <stdint.h>

#define DEBUG 1
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#define PRINT6ADDR(addr) PRINTF("[%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x]", ((uint8_t *)addr)[0], ((uint8_t *)addr)[1], ((uint8_t *)addr)[2], ((uint8_t *)addr)[3], ((uint8_t *)addr)[4], ((uint8_t *)addr)[5], ((uint8_t *)addr)[6], ((uint8_t *)addr)[7], ((uint8_t *)addr)[8], ((uint8_t *)addr)[9], ((uint8_t *)addr)[10], ((uint8_t *)addr)[11], ((uint8_t *)addr)[12], ((uint8_t *)addr)[13], ((uint8_t *)addr)[14], ((uint8_t *)addr)[15])
#define PRINTLLADDR(lladdr) PRINTF("[%02x:%02x:%02x:%02x:%02x:%02x]",(lladdr)->addr[0], (lladdr)->addr[1], (lladdr)->addr[2], (lladdr)->addr[3],(lladdr)->addr[4], (lladdr)->addr[5])
#else
#define PRINTF(...)
#define PRINT6ADDR(addr)
#define PRINTLLADDR(addr)
#endif


/*---------------------------------------------------------------------------*/
/* Packet sniffer */
static uint32_t packets_received = 0;
static uint32_t packets_transmitted = 0;

void packet_received(void) { packets_received++; }
void packet_transmitted(int mac_status) { packets_transmitted++; }

RIME_SNIFFER(packet_counter, &packet_received, &packet_transmitted);

/*---------------------------------------------------------------------------*/
/* CC2520 RX/TX time measurement variables */
// Note that all times are in timer ticks, usually of the rtimer. For the rm090 this is by default the Timer1_A0 timer
static volatile uint8_t cc2520_rxtx_status; // variable for persisting the RX and TX active bits from the cc2520 status bytes inbetween SFD ISR calls
static volatile uint16_t cc2520_sfd_start_time; // time (in rtimer ticks) when SFD comes high, measured in the ISR of the SFD pin (TODO: use interrupt of hardware timer for more precise value)
static volatile uint16_t cc2520_sfd_end_time; // time (in rtimer ticks) when SFD goes low, measured in the ISR of the SFD pin
static volatile uint16_t cc2520_sfd_rx_time; // Total time spent receiving a frame (i.e. idle listen time not included) in rtimer ticks
static volatile uint16_t cc2520_sfd_tx_time; // Total time spent actually transmitting a frame (this includes acks) in rtimer ticks
static volatile uint16_t cc2520_sfd_rx_counter; // Total number of frames received
static volatile uint16_t cc2520_sfd_tx_counter; // Total number of frames transmitted (incl. acks), look at RIME_SNIFFER if you don't want acks (unverified...)

/*---------------------------------------------------------------------------*/
PROCESS(cc2538_demo_process, "cc2538 demo process");
AUTOSTART_PROCESSES(&cc2538_demo_process);

void print_stats() {
  char buffer[40];

  // For writing 32-bit values, address should be even
  uint32_t *buffer_ = (uint32_t *)buffer;
  if((uint16_t)buffer % 2 == 1) {
    buffer_ = (uint32_t *)(buffer + 1);
  }

  buffer_[0] = clock_time();
  buffer_[1] = energest_type_time(ENERGEST_TYPE_CPU);
  buffer_[2] = energest_type_time(ENERGEST_TYPE_LPM);
  buffer_[5] = energest_type_time(ENERGEST_TYPE_IRQ);
  buffer_[3] = energest_type_time(ENERGEST_TYPE_TRANSMIT);
  buffer_[4] = energest_type_time(ENERGEST_TYPE_LISTEN);
  buffer_[6] = compower_idle_activity.transmit;
  buffer_[7] = compower_idle_activity.listen;
  buffer_[8] = packets_transmitted;
  buffer_[9] = packets_received;

  PRINTF("%lu %lu %lu %lu %lu %lu %lu %lu %lu %lu\n", buffer_[0], buffer_[1], buffer_[2], buffer_[3], buffer_[4], buffer_[5], buffer_[6], buffer_[7], buffer_[8], buffer_[9]);
  PRINTF("RX: %u packets\t %u ticks\nTX: %u packets\t %u ticks\n", cc2520_sfd_rx_counter, cc2520_sfd_rx_time, cc2520_sfd_tx_counter, cc2520_sfd_tx_time);
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(cc2538_demo_process, ev, data)
{
  PROCESS_BEGIN();

  PRINTF("PROCESS STARTED");

  /* initialize serial line */
  serial_line_init();
#ifdef CONTIKI_TARGET_RM090
  uart1_set_input(serial_line_input_byte);
#elif defined(CONTIKI_TARGET_Z1)
  uart0_set_input(serial_line_input_byte);
#endif

  // Energest:
  energest_init();

  // Rime sniffer:
  rime_sniffer_add(&packet_counter);

  // Configure CC2520's GPIO SFD pin on the msp430:
  P2REN |= 0x01;                            // Enable P2.0 internal resistance
  P2OUT |= 0x01;                            // Set P2.0 as pull-Up resistance

  P2IE |= 0x01;                             // P2.0 interrupt enabled
  //P2DIR &= ~0x01;                           // P2.0 as input pin?

  //Start with low to high edge:
  P2IES &= ~0x01;                            // P2.0 Lo/Hi edge
  P2IFG &= ~0x01;                           // P2.0 IFG cleared

  // Listen for serial event:
  while(1) {
    PROCESS_YIELD();

    if(ev == PROCESS_EVENT_TIMER) {
    } else if(ev == serial_line_event_message) {
      //char* line = (char *)data;
      //PRINTF("serial_line_event_message event received, data: %s\n", line);
      print_stats(); // always print stats for every serial line event
    }
  }

  PROCESS_END();
}

/* P2.0 of the rm090 is hooked up to cc2520 gpio p4, i.e. SFD:
 * Pin is high when SFD has been received or
 transmitted. Cleared when leaving RX/TX
 respectively.
 */
// Note: this ISR should only be called for when P2.0 changes state, not for any other pins on port 2 as the push button is also on this port, it should be disabled for now...
//ISR(PORT2, cc2520_port2_interrupt)
interrupt(CC2520_IRQ2_VECTOR)
cc2520_port2_interrupt(void)
{
  // Frist of all get the current timer value
  rtimer_clock_t clocktime = rtimer_arch_now();

  ENERGEST_ON(ENERGEST_TYPE_IRQ);

  if (CC2520_SFD_IS_1)
  {
    /* Start of SFD for RX or TX*/
    // Get the status byte via SPI from the CC2520:
    uint8_t status;
    CC2520_GET_STATUS(status);

    // Persist the status for when SFD goes low (note that the status byte is no longer usable when SFD is low):
    if ((status & 0x03) == 0x01) // currently receiving
      cc2520_rxtx_status = 1;
    else if ((status & 0x03) == 0x02) // currently transmitting
      cc2520_rxtx_status = 2;
    else // invalid...
      cc2520_rxtx_status = 0;

    // Save start time:
    cc2520_sfd_start_time = clocktime;

    // next time we want high to low edge:
    P2IES |= 0x01;                            // P2.0 Hi/Lo edge
  }
  else
  {
    /* Stop of SFD for RX or TX*/
    cc2520_sfd_end_time = clocktime;
    if (cc2520_rxtx_status == 1) { // last activity on the radio was RX:
      cc2520_sfd_rx_time += (cc2520_sfd_end_time - cc2520_sfd_start_time);
      cc2520_sfd_rx_counter++;
    } else if (cc2520_rxtx_status == 2) { // last activity on the radio was TX:
      cc2520_sfd_tx_time += (cc2520_sfd_end_time - cc2520_sfd_start_time);
      cc2520_sfd_tx_counter++;
    }
     // Clear status for next time:
    cc2520_rxtx_status = 0;

    // next time we want low to high edge:
    P2IES &= ~0x01;                            // P2.0 Lo/Hi edge
  }

  // Clear IV:
  P2IFG = 0x00;
  ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}

