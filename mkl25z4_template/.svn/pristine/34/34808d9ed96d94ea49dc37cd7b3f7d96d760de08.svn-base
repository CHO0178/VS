/*
 * Name: bsort_tb.c
 * Author: Martin Stankus
 *
 */

#include "wdog.h"

#include "bsort_ref.h"
#include "bsort.h"
#include "bsort_fptr.h"

int sort_up(int a, int b)
{
	if (a > b) {
		return 1;
	}

	return 0;
}

int sort_down(int a, int b)
{
	if (a < b) {
		return 1;
	}

	return 0;
}

int main(void)
{
	int data1[] = {5, 0, 9, 1, 6};
	int data2[] = {5, 0, 9, 1, 6};
	int data3[] = {5, 0, 9, 1, 6};

	wdog_init(WDOG_CONF_DIS);

	bsort_ref(data1, 5);
	bsort(data2, 5);
	bsort_fptr(data3, 5, sort_up);
	bsort_fptr(data3, 5, sort_down);

	while (1) {
		wdog_refresh();
	}

	return 0;
}
