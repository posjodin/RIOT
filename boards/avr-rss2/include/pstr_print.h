#ifndef PSTR_PRINT_H
#define PSTR_PRINT_H

#ifdef BOARD_AVR_RSS2
#include <avr/pgmspace.h>

#define printf(FMT, ...) printf_P(PSTR(FMT), ##__VA_ARGS__)   
#define snprintf(BUF, LEN, FMT, ...) snprintf_P((BUF), (LEN), PSTR(FMT), ##__VA_ARGS__)   
#define puts(STR) puts_P(PSTR(STR))
#endif

#endif /* PSTR_PRINT_H */
