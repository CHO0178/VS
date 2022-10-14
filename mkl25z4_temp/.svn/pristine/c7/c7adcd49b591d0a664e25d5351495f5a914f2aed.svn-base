/*
 * Name: print.c
 * Author: Martin Stankus
 *
 */

#include "print.h"
#include "print_priv.h"

#if PRINT_MMDVSQ_EN
#include "mmdvsq.h"
#endif

unsigned int print_udec(char *str, unsigned long num)
{
	char buf[PRINT_DEC_BUF_LEN];
	unsigned int ind_str, ind_buf = 0ul;

	buf[ind_buf++] = 0;

	do {
#if PRINT_MMDVSQ_EN
		buf[ind_buf++] = mmdvsq_udiv_rem(num, 10ul) + '0';
		num = mmdvsq_udiv_quot(num, 10ul);
#else
		buf[ind_buf++] = (num % 10ul) + '0';
		num /= 10ul;
#endif
	} while (num > 0ul);

	ind_buf--;

	for (ind_str = 0ul; ind_str <= ind_buf; ind_str++) {
		str[ind_str] = buf[ind_buf - ind_str];
	}

	return ind_buf;
}

unsigned int print_dec(char *str, long num)
{
	char buf[PRINT_DEC_BUF_LEN];
	unsigned int ind_str, ind_buf = 0ul;

	buf[ind_buf++] = 0;

	if (num >= 0l) {
		do {
#if PRINT_MMDVSQ_EN
		buf[ind_buf++] = mmdvsq_udiv_rem(num, 10ul) + '0';
		num = mmdvsq_udiv_quot(num, 10ul);
#else
		buf[ind_buf++] = (num % 10ul) + '0';
		num /= 10ul;
#endif
		} while (num > 0l);

		ind_buf--;
	} else {
		do {
#if PRINT_MMDVSQ_EN
			buf[ind_buf++] = -mmdvsq_div_rem(num, 10l) + '0';
			num = mmdvsq_div_quot(num, 10l);
#else
			buf[ind_buf++] = -(num % 10l) + '0';
			num /= 10l;
#endif
		} while (num < 0l);

		buf[ind_buf] = '-';
	}

	for (ind_str = 0ul; ind_str <= ind_buf; ind_str++) {
		str[ind_str] = buf[ind_buf - ind_str];
	}

	return ind_buf;
}

unsigned int print_uhex(char *str, unsigned char digit_cnt, unsigned long num)
{
	unsigned int shift, digit, ind_str = 0u;

	if (digit_cnt > 0u) {
		shift = (digit_cnt - 1u) * 4u;

		while (ind_str < digit_cnt) {
			digit = (num >> shift) & 0xFul;
			shift -= 4u;

			if (digit < 10u) {
				str[ind_str] = digit + '0';
			} else {
				str[ind_str] = digit + '7';
			}

			ind_str++;
		}
	}

	str[ind_str] = 0;

	return ind_str;
}
