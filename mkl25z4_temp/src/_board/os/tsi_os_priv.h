/*
 * Name: tsi_os_priv.h
 * Author: Martin Stankus
 *
 */

#ifndef _TSI_OS_PRIV_H_
#define _TSI_OS_PRIV_H_

#define TSI_GENCS_COMPVAL	(TSI_GENCS_ESOR_MASK | \
							TSI_GENCS_REFCHRG(7u) | \
							TSI_GENCS_DVOLT(3u) | \
							TSI_GENCS_EXTCHRG(7u) |\
							TSI_GENCS_PS(0u) |\
							TSI_GENCS_NSCN(31u) |\
							TSI_GENCS_TSIEN_MASK |\
							TSI_GENCS_TSIIEN_MASK)

static void __attribute__ ((interrupt)) tsi_os_scan_handler(void);

#endif /* _TSI_OS_PRIV_H_ */
