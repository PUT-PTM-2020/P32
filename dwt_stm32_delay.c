#include "dwt_stm32_delay.h"


/**
 * @brief  Initializes DWT_Clock_Cycle_Count for DWT_Delay_us function
 * @return Error DWT counter
 *         1: clock cycle counter not started
 *         0: clock cycle counter works
 */
uint32_t DWT_Delay_Init(void) {
  /* Disable TRC */
  CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk; // ~0x01000000;
  /* Enable TRC */
  CoreDebug->DEMCR |=  CoreDebug_DEMCR_TRCENA_Msk; // 0x01000000;

  /* Disable clock cycle counter */
  DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; //~0x00000001;
  /* Enable  clock cycle counter */
  DWT->CTRL |=  DWT_CTRL_CYCCNTENA_Msk; //0x00000001;

  /* Reset the clock cycle counter value */
  DWT->CYCCNT = 0;

     /* 3 NO OPERATION instructions */
     __ASM volatile ("NOP");
     __ASM volatile ("NOP");
  __ASM volatile ("NOP");

  /* Check if clock cycle counter has started */
     if(DWT->CYCCNT)
     {
       return 0; /*clock cycle counter started*/
     }
     else
  {
    return 1; /*clock cycle counter not started*/
  }
}

/* Use DWT_Delay_Init (); and DWT_Delay_us (microseconds) in the main */

/*
 * Simple microseconds delay routine, utilizing ARM's DWT
 * (Data Watchpoint and Trace Unit) and HAL library.
 * Intended to use with gcc compiler, but I hope it can be used
 * with any other C compiler across the Universe (provided that
 * ARM and CMSIS already invented) :)
 * Max K
 *
 *
 * This file is part of DWT_Delay package.
 * DWT_Delay is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * us_delay is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See
 * the GNU General Public License for more details.
 * http://www.gnu.org/licenses/.
 */

//#include "stm32f4xx_hal.h"          // change to whatever MCU you use
//#include "dwt_stm32_delay.h"
//
///**
// * Initialization routine.
// * You might need to enable access to DWT registers on Cortex-M7
// *   DWT->LAR = 0xC5ACCE55
// */
//void DWT_Init(void)
//{
//    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
//        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
//        DWT->CYCCNT = 0;
//        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
//    }
//}
//
//#if DWT_DELAY_NEWBIE
///**
// * If you are a newbie and see magic in DWT_Delay, consider this more
// * illustrative function, where you explicitly determine a counter
// * value when delay should stop while keeping things in bounds of uint32.
//*/
//void DWT_Delay(uint32_t us) // microseconds
//{
//    uint32_t startTick  = DWT->CYCCNT,
//             targetTick = DWT->CYCCNT + us * (SystemCoreClock/1000000);
//
//    // Must check if target tick is out of bounds and overflowed
//    if (targetTick > startTick) {
//        // Not overflowed
//        while (DWT->CYCCNT < targetTick);
//    } else {
//        // Overflowed
//        while (DWT->CYCCNT > startTick || DWT->CYCCNT < targetTick);
//    }
//}
//#else
///**
// * Delay routine itself.
// * Time is in microseconds (1/1000000th of a second), not to be
// * confused with millisecond (1/1000th).
// *
// * No need to check an overflow. Let it just tick :)
// *
// * @param uint32_t us  Number of microseconds to delay for
// */
//void DWT_Delay(uint32_t us) // microseconds
//{
//    uint32_t startTick = DWT->CYCCNT,
//             delayTicks = us * (SystemCoreClock/1000000);
//
//    while (DWT->CYCCNT - startTick < delayTicks);
//}
//
//#endif
