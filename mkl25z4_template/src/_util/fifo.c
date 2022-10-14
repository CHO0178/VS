/*
 * Name: fifo.c
 * Author: Martin Stankus
 *
 */

#include <stdint.h>

#include "fifo.h"
#include "fifo_priv.h"

#include "mem.h"

FIFO_RET fifo_init_var(FIFO_DESC_VAR *desc, void *base, uint32_t len, uint32_t wid)
{
	if (fifo_check_len(len)) {
		return FIFO_RET_ERR_PARAM;
	}

	fifo_adj_desc_init(&desc->desc_gen, len);
	desc->wid = wid;
	desc->base = base;

	return FIFO_RET_OK;
}

FIFO_RET fifo_ins_var(FIFO_DESC_VAR *desc, const void *data)
{
	if (desc->desc_gen.cnt == desc->desc_gen.len) {
		return FIFO_RET_ERR_FULL;
	}

	fifo_adj_desc_ins(&desc->desc_gen);

	memcpy(desc->base + (desc->desc_gen.ind_tail * desc->wid), data, desc->wid);

	return FIFO_RET_OK;
}

FIFO_RET fifo_ins_owr_head_var(FIFO_DESC_VAR *desc, const void *data)
{
	fifo_adj_desc_ins_owr_head(&desc->desc_gen);

	memcpy(desc->base + (desc->desc_gen.ind_tail * desc->wid), data, desc->wid);

	return FIFO_RET_OK;
}

FIFO_RET fifo_ins_owr_tail_var(FIFO_DESC_VAR *desc, const void *data)
{
	if (desc->desc_gen.cnt != desc->desc_gen.len) {
		fifo_adj_desc_ins(&desc->desc_gen);
	}

	memcpy(desc->base + (desc->desc_gen.ind_tail * desc->wid), data, desc->wid);

	return FIFO_RET_OK;
}

FIFO_RET fifo_rem_var(FIFO_DESC_VAR *desc, void *data)
{
	if (desc->desc_gen.cnt == 0ul) {
		return FIFO_RET_ERR_EMPTY;
	}

	memcpy(data, desc->base + (desc->desc_gen.ind_head * desc->wid), desc->wid);

	fifo_adj_desc_rem(&desc->desc_gen);

	return FIFO_RET_OK;
}

FIFO_RET fifo_init_byte(FIFO_DESC_BYTE *desc, uint8_t *base, uint32_t len)
{
	if (fifo_check_len(len)) {
		return FIFO_RET_ERR_PARAM;
	}

	fifo_adj_desc_init(&desc->desc_gen, len);
	desc->base = base;

	return FIFO_RET_OK;
}

FIFO_RET fifo_ins_byte(FIFO_DESC_BYTE *desc, uint8_t data)
{
	if (desc->desc_gen.cnt == desc->desc_gen.len) {
		return FIFO_RET_ERR_FULL;
	}

	fifo_adj_desc_ins(&desc->desc_gen);

	desc->base[desc->desc_gen.ind_tail] = data;

	return FIFO_RET_OK;
}

FIFO_RET fifo_ins_owr_head_byte(FIFO_DESC_BYTE *desc, uint8_t data)
{
	fifo_adj_desc_ins_owr_head(&desc->desc_gen);

	desc->base[desc->desc_gen.ind_tail] = data;

	return FIFO_RET_OK;
}

FIFO_RET fifo_ins_owr_tail_byte(FIFO_DESC_BYTE *desc, uint8_t data)
{
	if (desc->desc_gen.cnt != desc->desc_gen.len) {
		fifo_adj_desc_ins(&desc->desc_gen);
	}

	desc->base[desc->desc_gen.ind_tail] = data;

	return FIFO_RET_OK;
}

FIFO_RET fifo_rem_byte(FIFO_DESC_BYTE *desc, uint8_t *data)
{
	if (desc->desc_gen.cnt == 0ul) {
		return FIFO_RET_ERR_EMPTY;
	}

	*data = desc->base[desc->desc_gen.ind_head];

	fifo_adj_desc_rem(&desc->desc_gen);

	return FIFO_RET_OK;
}

FIFO_RET fifo_init_word(FIFO_DESC_WORD *desc, uint16_t *base, uint32_t len)
{
	if (fifo_check_len(len)) {
		return FIFO_RET_ERR_PARAM;
	}

	fifo_adj_desc_init(&desc->desc_gen, len);
	desc->base = base;

	return FIFO_RET_OK;
}

FIFO_RET fifo_ins_word(FIFO_DESC_WORD *desc, uint16_t data)
{
	if (desc->desc_gen.cnt == desc->desc_gen.len) {
		return FIFO_RET_ERR_FULL;
	}

	fifo_adj_desc_ins(&desc->desc_gen);

	desc->base[desc->desc_gen.ind_tail] = data;

	return FIFO_RET_OK;
}

FIFO_RET fifo_ins_owr_head_word(FIFO_DESC_WORD *desc, uint16_t data)
{
	fifo_adj_desc_ins_owr_head(&desc->desc_gen);

	desc->base[desc->desc_gen.ind_tail] = data;

	return FIFO_RET_OK;
}

FIFO_RET fifo_ins_owr_tail_word(FIFO_DESC_WORD *desc, uint16_t data)
{
	if (desc->desc_gen.cnt != desc->desc_gen.len) {
		fifo_adj_desc_ins(&desc->desc_gen);
	}

	desc->base[desc->desc_gen.ind_tail] = data;

	return FIFO_RET_OK;
}

FIFO_RET fifo_rem_word(FIFO_DESC_WORD *desc, uint16_t *data)
{
	if (desc->desc_gen.cnt == 0ul) {
		return FIFO_RET_ERR_EMPTY;
	}

	*data = desc->base[desc->desc_gen.ind_head];

	fifo_adj_desc_rem(&desc->desc_gen);

	return FIFO_RET_OK;
}

FIFO_RET fifo_init_dword(FIFO_DESC_DWORD *desc, uint32_t *base, uint32_t len)
{
	if (fifo_check_len(len)) {
		return FIFO_RET_ERR_PARAM;
	}

	fifo_adj_desc_init(&desc->desc_gen, len);
	desc->base = base;

	return FIFO_RET_OK;
}

FIFO_RET fifo_ins_dword(FIFO_DESC_DWORD *desc, uint32_t data)
{
	if (desc->desc_gen.cnt == desc->desc_gen.len) {
		return FIFO_RET_ERR_FULL;
	}

	fifo_adj_desc_ins(&desc->desc_gen);

	desc->base[desc->desc_gen.ind_tail] = data;

	return FIFO_RET_OK;
}

FIFO_RET fifo_ins_owr_head_dword(FIFO_DESC_DWORD *desc, uint32_t data)
{
	fifo_adj_desc_ins_owr_head(&desc->desc_gen);

	desc->base[desc->desc_gen.ind_tail] = data;

	return FIFO_RET_OK;
}

FIFO_RET fifo_ins_owr_tail_dword(FIFO_DESC_DWORD *desc, uint32_t data)
{
	if (desc->desc_gen.cnt != desc->desc_gen.len) {
		fifo_adj_desc_ins(&desc->desc_gen);
	}

	desc->base[desc->desc_gen.ind_tail] = data;

	return FIFO_RET_OK;
}

FIFO_RET fifo_rem_dword(FIFO_DESC_DWORD *desc, uint32_t *data)
{
	if (desc->desc_gen.cnt == 0ul) {
		return FIFO_RET_ERR_EMPTY;
	}

	*data = desc->base[desc->desc_gen.ind_head];

	fifo_adj_desc_rem(&desc->desc_gen);

	return FIFO_RET_OK;
}

static uint8_t fifo_check_len(uint32_t len)
{
	uint32_t cnt, aux;

	if (len == 1ul) {
		return 1u;
	}

	aux = 1;
	for (cnt = 0ul; cnt < (sizeof(len) * 8u); cnt++) {
		if (len == aux) {
			return 0u;
		}
		aux <<= 1u;
	}

	return 1u;
}

static void fifo_adj_desc_init(FIFO_DESC_GEN *desc_gen, uint32_t len)
{
	desc_gen->cnt = 0ul;
	desc_gen->ind_head = 0ul;
	desc_gen->ind_tail = 0ul;
	desc_gen->len = len;
	desc_gen->len_mask = len - 1ul;
}

static void fifo_adj_desc_ins(FIFO_DESC_GEN *desc_gen)
{
	if (desc_gen->cnt != 0ul) {
		desc_gen->ind_tail++;
		desc_gen->ind_tail &= desc_gen->len_mask;
	}

	desc_gen->cnt++;
}

static void fifo_adj_desc_ins_owr_head(FIFO_DESC_GEN *desc_gen)
{
	if (desc_gen->cnt == desc_gen->len) {
		desc_gen->ind_tail++;
		desc_gen->ind_tail &= desc_gen->len_mask;
		desc_gen->ind_head++;
		desc_gen->ind_head &= desc_gen->len_mask;
	} else if (desc_gen->cnt == 0ul) {
		desc_gen->cnt++;
	} else {
		desc_gen->ind_tail++;
		desc_gen->ind_tail &= desc_gen->len_mask;
		desc_gen->cnt++;
	}
}

static void fifo_adj_desc_rem(FIFO_DESC_GEN *desc_gen)
{
	if (desc_gen->cnt != 1ul) {
		desc_gen->ind_head++;
		desc_gen->ind_head &= desc_gen->len_mask;
	}

	desc_gen->cnt--;
}
