/*
 * Name: assert.h
 * Author: Martin Stankus
 *
 */

#ifndef _ASSERT_H_
#define _ASSERT_H_

#ifndef ASSERT_BKPT
#define ASSERT_BKPT		0
#endif

#if ASSERT_BKPT

#define assert(x)		if((x) == 0) {										\
							__asm volatile ("cpsid i");						\
							while (1) {										\
								__asm volatile ("bkpt");					\
							}												\
						}

#else

#define assert(x)		if((x) == 0) {										\
							__asm volatile ("cpsid i");						\
							while (1);										\
						}

#endif

#endif /* _ASSERT_H_ */
