/*
 * Name: mem.h
 * Author: Martin Stankus
 *
 */

#ifndef _MEM_H_
#define _MEM_H_

int memcmp(const void *src1, const void *src2, unsigned int len);
void* memcpy(void *dest, const void *src, unsigned int len);
void* memmove(void *dest, const void *src, unsigned int len);
void* memset(void *dest, int val, unsigned int len);

#endif /* _MEM_H_ */
