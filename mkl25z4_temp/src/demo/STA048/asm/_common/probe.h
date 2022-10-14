/*
 * Name: probe.h
 * Author: Martin Stankus
 *
 */

#ifndef _PROBE_H_
#define _PROBE_H_

#include <stdint.h>

void probe_hardfault_handler(void);
uint32_t probe(uint8_t *dst_addr, uint8_t *src_addr);

#endif /* _PROBE_H_ */
