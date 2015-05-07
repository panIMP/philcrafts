#include "../inc/error.h"

void* malloc_check(size_t _Size)
{
	void* p = malloc(_Size);

	if (p == NULL)
	{
		DEBUG_PRINT_DETAILED("Memory allocation failed!");
	}

	return p;
}

void* calloc_check(size_t _NumOfElements, size_t _SizeOfElement)
{
	void* p = calloc(_NumOfElements, _SizeOfElement);

	if (p == NULL)
	{
		DEBUG_PRINT_DETAILED("Memory allocation failed!");
	}

	return p;
}