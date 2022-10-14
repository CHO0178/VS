/*
 * Name: pool.h
 * Author: Martin Stankus
 *
 */

#ifndef _POOL_H_
#define _POOL_H_

#include <stdint.h>

typedef struct {
	uint32_t cnt;
	uint32_t ind_head;
	uint32_t ind_tail;
	uint32_t len;
	uint32_t len_mask;
	uint32_t *base;
} POOL_DESC;

typedef enum {
	POOL_RET_OK = 0,
	POOL_RET_ERR_FULL,
	POOL_RET_ERR_EMPTY,
	POOL_RET_ERR_PARAM
} POOL_RET;

POOL_RET pool_init(POOL_DESC *desc, uint32_t *base, uint32_t len);
POOL_RET pool_alloc(POOL_DESC *desc, uint32_t *token);
POOL_RET pool_free(POOL_DESC *desc, uint32_t token);

#endif /* _POOL_H_ */
