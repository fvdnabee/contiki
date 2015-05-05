#ifndef __TCPIP_ERBR_H__
#define __TCPIP_ERBR_H__

#include "contiki.h"
#include "uip.h"

void set_last_sender(uip_ipaddr_t *ls);
uint8_t is_last_sender(uip_ipaddr_t *addr);

#endif
