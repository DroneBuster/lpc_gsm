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

#include <sim900d.h>

#define SIM900_DETECTED 0x01

uint8_t g_boardStatus = 0;

static const SerialConfig uart2_cfg = {
    115200,
    LCR_WL8 | LCR_STOP1 | LCR_NOPARITY,
    FCR_TRIGGER0
};
static const SerialConfig uart3_cfg = {
    57600,
    LCR_WL8 | LCR_STOP1 | LCR_NOPARITY,
    FCR_TRIGGER0
};

static WORKING_AREA(waBlinkerThread, 64);
static msg_t BlinkerThread(void *arg) {
  (void)arg;
  palClearPad(GPIO1, GPIO1_LED2_Y);
  palClearPad(GPIO1, GPIO1_LED1_Y);
  while (TRUE) {
    systime_t time = 50;
    if (g_boardStatus & SIM900_DETECTED) {
      time = 250;
    }
    palTogglePad(GPIO1, GPIO1_LED2_Y);
    chThdSleepMilliseconds(time);
  }
  /* This point may be reached if shut down is requested. */
  return 0;
}

static WORKING_AREA(waUARTResend, 2048);
static msg_t UARTResend(void *arg) {
  (void)arg;
  uint8_t buf[64];
  while (TRUE) {
    uint8_t bytesRead = chnReadTimeout(&SD3, buf, 64, MS2ST(5));
    if(bytesRead > 0)
    chnWrite(&SD4, buf, bytesRead);
    bytesRead = chnReadTimeout(&SD4, buf, 64, MS2ST(5));
    if(bytesRead > 0)
    chnWrite(&SD3, buf, bytesRead);
    //chnWrite(&SD4, b, sizeof(b));
    chThdSleepMilliseconds(100);
  }
  /* This point may be reached if shut down is requested. */
  return 0;
}

/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  reset_sim900d();

  sdStart(&SD3, &uart2_cfg); //UART2 for GSM
  sdStart(&SD4, &uart3_cfg); //UART3 for telemetry

  chThdCreateStatic(waUARTResend, sizeof(waUARTResend),
        NORMALPRIO - 1, UARTResend, NULL);

  chThdCreateStatic(waBlinkerThread, sizeof(waBlinkerThread),
      NORMALPRIO - 1, BlinkerThread, NULL);

  while (TRUE) {

      chThdSleepMilliseconds(500);
    }

}
