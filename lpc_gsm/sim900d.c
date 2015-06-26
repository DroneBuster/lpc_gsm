#include "ch.h"
#include "hal.h"

#include <sim900d.h>

void reset_sim900d(void) {
  palClearPad(GPIO2, GPIO2_RES_LEA);
  chThdSleepMilliseconds(500);
  palSetPad(GPIO2, GPIO2_RES_LEA);
}
