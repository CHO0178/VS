/*
 * Name: pool.c
 * Author: Martin Stankus
 *
 */

#include <stdint.h>

#include "pool.h"
#include "pool_priv.h"

POOL_RET pool_init(POOL_DESC *desc, uint32_t *base, uint32_t len)
{
	uint32_t ind;

	if (pool_check_len(len)) {
		return POOL_RET_ERR_PARAM;
	}

	for (ind = 0ul; ind < len; ind++) {
		base[ind] = ind;
	}

	desc->cnt = len;
	desc->ind_head = 0ul;
	desc->ind_tail = len - 1ul;
	desc->len = len;
	desc->len_mask = len - 1ul;
	desc->base = base;

	return POOL_RET_OK;
}

POOL_RET pool_alloc(POOL_DESC *desc, uint32_t *token)
{
	if (desc->cnt == 0ul) {
		return POOL_RET_ERR_EMPTY;
	}

	*token = desc->base[desc->ind_head];

	if (desc->cnt != 1ul) {
		desc->ind_head++;
		desc->ind_head &= desc->len_mask;
	}

	desc->cnt--;

	return POOL_RET_OK;
}

POOL_RET pool_free(POOL_DESC *desc, uint32_t token)
{
	if (desc->cnt == desc->len) {
		return POOL_RET_ERR_FULL;
	}

	if (desc->cnt != 0ul) {
		desc->ind_tail++;
		desc->ind_tail &= desc->len_mask;
	}

	desc->cnt++;

	desc->base[desc->ind_tail] = token;

	return POOL_RET_OK;
}

static uint8_t pool_check_len(uint32_t len)
{
	uint32_t cnt, aux;

	if (len == 1ul) {
		return 1u;
	}

	aux = 1ul;
	for (cnt = 0; cnt < (sizeof(len) * 8u); cnt++) {
		if (len == aux) {
			return 0u;
		}
		aux <<= 1u;
	}

	return 1u;
}
