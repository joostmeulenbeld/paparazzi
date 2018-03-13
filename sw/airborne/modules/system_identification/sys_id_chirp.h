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
 * @file "modules/helicopter/sys_id_chirp.h"
 * @author Joost Meulenbeld
 * 
 * System identification chirp
 * 
 * A "chirp" or frequency sweep is a sine wave with in time increasing frequency, and can be
 * used for system identification purposes. Using the output of the chirp function
 * as input to the system allows registering the system behavior over a broad frequency
 * spectrum, while making sure that all frequencies are actually encountered.
 *
 * Chirps can be made with frequency increasing linearly or exponentially with time. This is set using an argument in
 * the chirp_init method. The latter one is better for system identification, according to [2].
 * 
 * See [1] for a simple derivation of the linear-time chirp and some nice figures; See [2] for more information regarding
 * execution. Some tips from that book: 
 *  - Start and stop with a couple of seconds at the trim state to allow bias removals from the data
 *  - Make sure the aircraft stays in the vicinity of trim condition
 *  - Add noise to the on-axis and off-axis inputs (0.1 times amplitude on-axis, 0.2 times amplitude off-axis)
 *  - Minimum length of the chirp 4 times wavelength of lowest frequency
 *
 * Features of this module:
 *  - Start/stop frequency and chirp length are configurable during flight, (but don't change them while doing a chirp)
 *  - If fade_in is set to true, the chirp starts with a gradual increase in amplitude on the lowest frequency,
 *      making it easier to maintain the uav at the current position
 *  - Noise is configurable separately for on-axis and off-axis inputs. The noise is a gaussian that is first-order
 *      filtered with a cutoff-frequency of the chirp maximum frequency
 * However, it takes up a fair amount of the total chirp time, so adjust your length_s accordingly!
 *
 * [1] https://en.wikipedia.org/wiki/Chirp for a derivation of the linear chirp
 * [2] Aircraft and Rotorcraft System Identification, 2nd edition by M. Tischler for a derivation of exponential chirp
 */


#ifndef SYS_ID_CHIRP_H
#define SYS_ID_CHIRP_H

#include <std.h>
#include <stdbool.h>
#include "modules/system_identification/pprz_chirp.h"
#include "generated/airframe.h"
#include "mcu_periph/sys_time.h"

// Number of axes for which the chirp will generate a signal
#define CHIRP_NO_AXES 3

// The current values for all axes that should be added to output commands to the UAV
extern int32_t current_chirp_values[CHIRP_NO_AXES];

extern uint8_t chirp_active;
extern int32_t chirp_amplitude;
extern float chirp_noise_stdv_onaxis_ratio;
extern float chirp_noise_stdv_offaxis;

extern float chirp_f0_hz;
extern float chirp_f1_hz;
extern float chirp_length_s;

extern uint8_t chirp_axis;

void sys_id_chirp_init(void);

// If chirp is running, update its values
extern void sys_id_chirp_run(void);

// Handler for changing the chirp_active variable in the GCS
extern void sys_id_chirp_chirp_activate_handler(uint8_t activate);

#endif // SYS_ID_CHIRP_H
