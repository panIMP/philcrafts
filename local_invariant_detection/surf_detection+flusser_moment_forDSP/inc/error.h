#ifndef ERROR_H
#define ERROR_H

#include <stdio.h>
#include <malloc.h>
#include <stdlib.h>


void* malloc_check(size_t _Size);
void* calloc_check(size_t _NumOfElements, size_t _SizeOfElement);


#ifdef _DEBUG_INFO_
#define DEBUG_PRINT_DETAILED(format, ...) printf("\n\nFile: " __FILE__ "\n" "Line: %d\nMsgs: " format, __LINE__, ##__VA_ARGS__)
#define DEBUG_PRINT_SIMPLIFIED(format, ...) printf(format, ##__VA_ARGS__)

#else
#define DEBUG_PRINT_DETAILED(format, ...)
#define DEBUG_PRINT_SIMPLIFIED(format, ...)

#endif

#endif
