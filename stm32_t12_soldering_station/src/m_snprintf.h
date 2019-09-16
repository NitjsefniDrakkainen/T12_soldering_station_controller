/*
 * m_snprintf.h
 *
 *  Created on: 16.09.2019
 *      Author: NitjsefniDrakkainen
 */

#ifndef __M_SNPRINTF__
#define __M_SNPRINTF__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdarg.h>

int m_vsnprintf(char* buffer, unsigned int buffer_len, const char *fmt, va_list va);
int m_snprintf(char* buffer, unsigned int buffer_len, const char *fmt, ...);
void debug_print(USART_TypeDef* USARTx, const char *fmt, ...);
#ifdef __cplusplus
}
#endif

#define vsnprintf m_vsnprintf
#define snprintf m_snprintf

#define printf debug_print

#endif
