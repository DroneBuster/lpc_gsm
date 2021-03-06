/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012,2013 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file    ARM/AT91SAM7/armparams.h
 * @brief   ARM7 AT91SAM7 Specific Parameters.
 *
 * @defgroup ARM_AT91SAM7 AT91SAM7 Specific Parameters
 * @ingroup ARM_SPECIFIC
 * @details This file contains the ARM specific parameters for the
 *          AT91SAM7 platform.
 * @{
 */

#ifndef _ARMPARAMS_H_
#define _ARMPARAMS_H_

/**
 * @brief   ARM core model.
 */
#define ARM_CORE                ARM_CORE_ARM7TDMI

/**
 * @brief   AT91SAM7-specific wait for interrupt.
 * @details This implementation writes 1 into the PMC_SCDR register.
 */
#if !defined(port_wait_for_interrupt) || defined(__DOXYGEN__)
#if ENABLE_WFI_IDLE || defined(__DOXYGEN__)
#define port_wait_for_interrupt() {                                         \
  (*((volatile uint32_t *)0xFFFFFC04)) = 1;                                 \
}
#else
#define port_wait_for_interrupt()
#endif
#endif

#endif /* _ARMPARAMS_H_ */

/** @} */
