// Copyright [2015] <Phil Hu>
// basic c types redefinition, wrap, debug and error functions.

#ifndef ALG_BASE_H
#define ALG_BASE_H

#include <cstdio>
#include <cstdlib>
#include <cstring>

typedef short int16;
typedef int int32;
typedef char int8;
typedef unsigned char uint8;

void* malloc_check(size_t _Size);
void* calloc_check(size_t _NumOfElements, size_t _SizeOfElement);

#ifdef SHOW_DEBUG_INFO


#define DEBUG_PRINT_DETAILED(format, ...) printf("\n\nFile: " __FILE__ "\n" \
"Line: %d\nMsgs: " format, __LINE__, ##__VA_ARGS__)
#define DEBUG_PRINT_SIMPLIFIED(format, ...) printf(format, ##__VA_ARGS__)

#else
#define DEBUG_PRINT_DETAILED(format, ...)
#define DEBUG_PRINT_SIMPLIFIED(format, ...)
#endif  // SHOW_DEBUG_INFO

#endif  // ALG_TYPES_H
