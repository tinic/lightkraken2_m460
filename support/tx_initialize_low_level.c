/*
MIT License

Copyright (c) 2023 Tinic Uro

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#include "NuMicro.h"
#include "tx_api.h"

#include <ctype.h>

// Threadx variables
extern void *_tx_initialize_unused_memory;
extern void *_tx_thread_system_stack_ptr;

// Linker script symbols
extern void *__HeapLimit;
// CMSIS Wrapper points to `__StackTop` in the linker script
extern uint32_t __StackTop;

void _tx_initialize_low_level(void) {
  // Disable all the interrupts
  __disable_irq();

  // Set base of available memory to end of non-initialised RAM area
  size_t ram_segment_used_end_address = (size_t)&__HeapLimit;
  ram_segment_used_end_address += 4;
  _tx_initialize_unused_memory = (void *)ram_segment_used_end_address;

  // Setup Vector Table Offset
  // This is where the Vector table resides
  // * NOTE, We can also omit this since it has been added to `SystemInit`
  SCB->VTOR = FLASH_BASE;

  // Set system stack pointer from vector value
  // __INITIAL_SP is defined above (CMSIS usage)
  // We can also get this value from the linker script __StackTop
  _tx_thread_system_stack_ptr = (void *)&__StackTop;

  // Enable the cycle count register
  // 0 -> CYCCNTENA
  // ARMv7-M Architecture Reference Manual: Page C1-737
  DWT->CTRL |= (1 << 0);

  // Use timer instead of SysTick for threadx
  TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, TX_TIMER_TICKS_PER_SECOND);
  TIMER_EnableInt(TIMER0);
  NVIC_EnableIRQ(TMR0_IRQn);
  TIMER_Start(TIMER0);

  // Configure handler priorities
  // TODO, Find a better way to do this
  SCB->SHP[0] = 0;
  SCB->SHP[1] = 0;
  SCB->SHP[2] = 0;
  // SCB->SHP[3] = 0; // reserved

  // SCB->SHP[4] = 0; // reserved
  // SCB->SHP[5] = 0; // reserved
  // SCB->SHP[6] = 0; // reserved
  SCB->SHP[7] = 0xFF;

  // SCB->SHP[8] = 0; // reserved
  // SCB->SHP[9] = 0; // reserved
  SCB->SHP[10] = 0xFF;
  SCB->SHP[11] = 0x40;
}

// Timer interrupt handler

extern void _tx_timer_interrupt();

void TMR0_IRQHandler(void) {
    if(TIMER_GetIntFlag(TIMER0) == 1) {
        TIMER_ClearIntFlag(TIMER0);
#ifdef TX_ENABLE_EXECUTION_CHANGE_NOTIFY
        _tx_execution_isr_enter();
#endif
        _tx_timer_interrupt();
#ifdef TX_ENABLE_EXECUTION_CHANGE_NOTIFY
        _tx_execution_isr_exit();
#endif
    }
}

