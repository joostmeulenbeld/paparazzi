/*
 * Copyright (C) Joost Meulenbeld
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * General chirp mathematical equations implementation - sys_id_chirp.h contains the module code that implements
 * pprz_chirp.h/c's chirp generation. All functions in this file therefore require that you supply the current time
 * - This makes it easy to test all equations.
 */

#ifndef PPRZ_CHIRP_H
#define PPRZ_CHIRP_H

#include <math.h>
#include <std.h>
#include <stdbool.h>
#include <stdlib.h>
#include "mcu_periph/sys_time.h"

// Values for exponential chirp. C2 is based on C1 s.t. the frequency range exactly covers the required range
#define chirp_C1 4.0f
#define chirp_C2 1.0f / (exp(chirp_C1) - 1)

/**
 * Initialize/start with chirp_init
 */
struct chirp_t {
  float f0_hz;
  float f1_hz;
  float start_time_s; // System time at start of the chirp
  float length_s; // Amount of seconds of the chirp excluding fade-in if applicable
  float total_length_s; // Amount of seconds of the chirp including fade-in if applicable

  float current_frequency_hz;
  float current_value; // Current output value in [-1, 1]
  float current_time_s;
  float percentage_done;

  bool exponential_chirp; // from module settings
  bool fade_in; // from module settings
};

/**
 * Allocate and initialize a new chirp struct. set start_time to the current time
 * @param f0_hz: Minimum frequency of the chirp in Hz
 * @param f1_hz: Maximum frequency of the chirp in Hz
 * @param length_s: Time interval in s (starting from start_time_s) in which the chirp should be carried out, excluding fade-in time if applicable
 * @param current_time_s: Current time in s, starting point of the chirp
 * @param exponential_chirp: If true, use exponential-time chirp, otherwise use linear-time chirp (see wikipedia)
 * @param fade_in: If true, begin the chirp with 2 wavelengths of the lowest frequency, with increasing amplitude
 */
void chirp_init(struct chirp_t* chirp, float f0_hz, float f1_hz, float length_s, float current_time_s, bool exponential_chirp, bool fade_in);

/**
 * Reset the chirp
 * @param chirp: The chirp struct pointer to reset
 * @param current_time_s: The time to set the chirp start at
 **/
void chirp_reset(struct chirp_t* chirp, float current_time_s);

/**
 * Return if the current_time is within the chirp manoeuvre
 */
bool chirp_is_running(struct chirp_t* chirp, float current_time_s);

/**
 * Calculate the value at current_time_s and update the struct with current frequency and value
 * @return Current value chirp->current_value
 */
float chirp_update(struct chirp_t* chirp, float current_time_s);

#endif
