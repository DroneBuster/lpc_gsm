/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "ch.h"
#include "hal.h"

/**
 * @brief   PAL setup.
 * @details Digital I/O ports static configuration as defined in @p board.h.
 *          This variable is used by the HAL when initializing the PAL driver.
 */
#if HAL_USE_PAL || defined(__DOXYGEN__)
const PALConfig pal_default_config = {
 {VAL_GPIO0DATA, VAL_GPIO0DIR},
 {VAL_GPIO1DATA, VAL_GPIO1DIR},
 {VAL_GPIO2DATA, VAL_GPIO2DIR},
 {VAL_GPIO3DATA, VAL_GPIO3DIR},
 {VAL_GPIO4DATA, VAL_GPIO4DIR}
};
#endif

/*
 * Early initialization code.
 * This initialization must be performed just after stack setup and before
 * any other initialization.
 */
void __early_init(void) {

  LPC17xx_clock_init();
}

/*
 * Board-specific initialization code.
 */
void boardInit(void) {

  /*
   * Extra, board-specific, initializations.
   */
  //UART2 pin config
  LPC_PINCON->PINSEL4  |= (2UL << 16)| (2UL << 18);     /* Set UART2 TXD2 P2.8 and RXD2 P2.9 pins */
  LPC_PINCON->PINMODE4 |= (2UL << 16)| (2UL << 18);     /* Disable pull-up on UART2 pins */

  //UART3 pin config
  LPC_PINCON->PINSEL9  |= (3UL << 24)| (3UL << 26);     /* Set UART3 TXD3 P4.28 and RXD3 P2.29 */
  LPC_PINCON->PINMODE9 |= (2UL << 24)| (2UL << 26);     /* Disable pull-up on UART3 pins */





}
