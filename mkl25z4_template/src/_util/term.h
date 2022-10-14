/*
 * Name: term.h
 * Author: Martin Stankus
 *
 */

#ifndef _TERM_H_
#define _TERM_H_

//sgr arg - all attributes off
#define TERM_SGR_ARG_RESET				0

//sgr arg - text intensity
#define TERM_SGR_ARG_BOLD_ON			1
#define TERM_SGR_ARG_BOLD_OFF			22

//sgr arg - text underlining
#define TERM_SGR_ARG_ULINE_ON			4
#define TERM_SGR_ARG_ULINE_OFF			24

//sgr arg - fgc/bgc inversion
#define TERM_SGR_ARG_INV_ON				7
#define TERM_SGR_ARG_INV_OFF			27

//sgr arg - foreground color
#define TERM_SGR_ARG_FGC_BLACK			30
#define TERM_SGR_ARG_FGC_RED			31
#define TERM_SGR_ARG_FGC_GREEN			32
#define TERM_SGR_ARG_FGC_YELLOW			33
#define TERM_SGR_ARG_FGC_BLUE			34
#define TERM_SGR_ARG_FGC_MAGENTA		35
#define TERM_SGR_ARG_FGC_CYAN			36
#define TERM_SGR_ARG_FGC_WHITE			37

//sgr arg - background color
#define TERM_SGR_ARG_BGC_BLACK			40
#define TERM_SGR_ARG_BGC_RED			41
#define TERM_SGR_ARG_BGC_GREEN			42
#define TERM_SGR_ARG_BGC_YELLOW			43
#define TERM_SGR_ARG_BGC_BLUE			44
#define TERM_SGR_ARG_BGC_MAGENTA		45
#define TERM_SGR_ARG_BGC_CYAN			46
#define TERM_SGR_ARG_BGC_WHITE			47

//ed arg
#define TERM_ED_ARG_CUR_TO_END			0
#define TERM_ED_ARG_BEG_TO_CUR			1
#define TERM_ED_ARG_ALL					2

//el arg
#define TERM_EL_ARG_CUR_TO_END			0
#define TERM_EL_ARG_BEG_TO_CUR			1
#define TERM_EL_ARG_ALL					2

#define TERM_CSI						"\x1B["

#define TERM_CUP_EVAL(X,Y)				TERM_CSI #X ";" #Y "H"
#define TERM_SGR_EVAL(ARG)				TERM_CSI #ARG "m"
#define TERM_ED_EVAL(ARG)				TERM_CSI #ARG "J"
#define TERM_EL_EVAL(ARG)				TERM_CSI #ARG "K"

//cursor position
#define TERM_CUP(X,Y)					TERM_CUP_EVAL(X,Y)

//select graphic rendition
#define TERM_SGR(ARG)					TERM_SGR_EVAL(ARG)

//erase in display
#define TERM_ED(ARG)					TERM_ED_EVAL(ARG)

//erase in line
#define TERM_EL(ARG)					TERM_EL_EVAL(ARG)

//cursor off
#define TERM_CU_OFF						TERM_CSI "?25l"

//cursor on
#define TERM_CU_ON						TERM_CSI "?25h"

#endif /* _TERM_H_ */
