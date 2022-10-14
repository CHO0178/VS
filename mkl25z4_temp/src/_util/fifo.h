/*
 * Name: fifo.h
 * Author: Martin Stankus
 *
 */

#ifndef _FIFO_H_
#define _FIFO_H_

#include <stdint.h>

typedef struct {
	uint32_t cnt;
	uint32_t ind_head;
	uint32_t ind_tail;
	uint32_t len;
	uint32_t len_mask;
} FIFO_DESC_GEN;

typedef struct {
	FIFO_DESC_GEN desc_gen;
	uint32_t wid;
	void *base;
} FIFO_DESC_VAR;

typedef struct {
	FIFO_DESC_GEN desc_gen;
	uint8_t *base;
} FIFO_DESC_BYTE;

typedef struct {
	FIFO_DESC_GEN desc_gen;
	uint16_t *base;
} FIFO_DESC_WORD;

typedef struct {
	FIFO_DESC_GEN desc_gen;
	uint32_t *base;
} FIFO_DESC_DWORD;

typedef enum {
	FIFO_RET_OK = 0,
	FIFO_RET_ERR_FULL,
	FIFO_RET_ERR_EMPTY,
	FIFO_RET_ERR_PARAM
} FIFO_RET;

FIFO_RET fifo_init_var(FIFO_DESC_VAR *desc, void *base, uint32_t len, uint32_t wid);
FIFO_RET fifo_ins_var(FIFO_DESC_VAR *desc, const void *data);
FIFO_RET fifo_ins_owr_head_var(FIFO_DESC_VAR *desc, const void *data);
FIFO_RET fifo_ins_owr_tail_var(FIFO_DESC_VAR *desc, const void *data);
FIFO_RET fifo_rem_var(FIFO_DESC_VAR *desc, void *data);

FIFO_RET fifo_init_byte(FIFO_DESC_BYTE *desc, uint8_t *base, uint32_t len);
FIFO_RET fifo_ins_byte(FIFO_DESC_BYTE *desc, uint8_t data);
FIFO_RET fifo_ins_owr_head_byte(FIFO_DESC_BYTE *desc, uint8_t data);
FIFO_RET fifo_ins_owr_tail_byte(FIFO_DESC_BYTE *desc, uint8_t data);
FIFO_RET fifo_rem_byte(FIFO_DESC_BYTE *desc, uint8_t *data);

FIFO_RET fifo_init_word(FIFO_DESC_WORD *desc, uint16_t *base, uint32_t len);
FIFO_RET fifo_ins_word(FIFO_DESC_WORD *desc, uint16_t data);
FIFO_RET fifo_ins_owr_head_word(FIFO_DESC_WORD *desc, uint16_t data);
FIFO_RET fifo_ins_owr_tail_word(FIFO_DESC_WORD *desc, uint16_t data);
FIFO_RET fifo_rem_word(FIFO_DESC_WORD *desc, uint16_t *data);

FIFO_RET fifo_init_dword(FIFO_DESC_DWORD *desc, uint32_t *base, uint32_t len);
FIFO_RET fifo_ins_dword(FIFO_DESC_DWORD *desc, uint32_t data);
FIFO_RET fifo_ins_owr_head_dword(FIFO_DESC_DWORD *desc, uint32_t data);
FIFO_RET fifo_ins_owr_tail_dword(FIFO_DESC_DWORD *desc, uint32_t data);
FIFO_RET fifo_rem_dword(FIFO_DESC_DWORD *desc, uint32_t *data);

#endif /* _FIFO_H_ */
