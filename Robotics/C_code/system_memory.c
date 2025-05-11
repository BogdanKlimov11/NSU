/**
  ******************************************************************************
  * @file    system_memory.c
  * @brief   System memory management for STM32CubeIDE.
  * @author  STMicroelectronics
  * @date    2025
  *
  * @attention
  * Copyright (c) 2025 STMicroelectronics. All rights reserved.
  * Licensed under BSD 3-Clause license. See opensource.org/licenses/BSD-3-Clause.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <errno.h>
#include <stdint.h>

/* Variables -----------------------------------------------------------------*/
static uint8_t *s_heap_end = NULL;

/**
  * @brief  Allocates memory to the heap for Newlib.
  * @param  incr Memory size to allocate
  * @retval Pointer to allocated memory or NULL on error
  */
void *_sbrk(ptrdiff_t incr)
{
    extern uint8_t _end; /* Defined in linker script */
    extern uint8_t _estack; /* Defined in linker script */
    extern uint32_t _Min_Stack_Size; /* Defined in linker script */
    const uint32_t stack_limit = (uint32_t)&_estack - (uint32_t)&_Min_Stack_Size;
    const uint8_t *max_heap = (uint8_t *)stack_limit;
    uint8_t *prev_heap_end;

    if (s_heap_end == NULL) {
        s_heap_end = &_end;
    }

    if (s_heap_end + incr > max_heap) {
        errno = ENOMEM;
        return (void *)-1;
    }

    prev_heap_end = s_heap_end;
    s_heap_end += incr;

    return (void *)prev_heap_end;
}
