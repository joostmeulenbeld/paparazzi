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
 * @file "modules/helicopter/sys_id_chirp.c"
 * @author Joost Meulenbeld
 * System identification chirp
 */

#include "modules/system_identification/sys_id_chirp.h"
#include "subsystems/datalink/telemetry.h"
#include "random.h"
#include "filters/low_pass_filter.h"

static struct chirp_t chirp;
uint8_t chirp_active = false;
uint8_t chirp_axis = 0;
int32_t chirp_amplitude = 0;
float chirp_noise_stdv_onaxis_ratio = 0.1;
float chirp_noise_stdv_offaxis = 100;
float chirp_f0_hz = 0.05;
float chirp_f1_hz = 3.2;
float chirp_length_s = 80;

// Filters used to cut-off the gaussian noise fed into the identification channels
static struct FirstOrderLowPass filters[CHIRP_NO_AXES];

int32_t current_chirp_values[];

static void set_current_chirp_values(void) {
  if (chirp_active) {
    // Loop over the chirps axes and add noise 
    for (uint8_t i = 0; i < CHIRP_NO_AXES; i++) {
      update_first_order_low_pass(&filters[i], rand_gaussian());
      if (chirp_axis == i) {
        current_chirp_values[i] = chirp.current_value;
        float noise_amplitude = chirp_amplitude * chirp_noise_stdv_onaxis_ratio;
        current_chirp_values[i] += (int32_t) (noise_amplitude * get_first_order_low_pass(&filters[i]));
      }
      else
        current_chirp_values[i] = (int32_t) (chirp_noise_stdv_offaxis * get_first_order_low_pass(&filters[i]));
    }
  }
  else {
    for (uint8_t i = 0; i < CHIRP_NO_AXES; i++)
      current_chirp_values[i] = 0;
  }
}

static void send_chirp(struct transport_tx *trans, struct link_device *dev) {
  pprz_msg_send_CHIRP(trans, dev, AC_ID, &chirp_active, &chirp.percentage_done, &chirp_axis, &chirp.current_frequency_hz, &chirp.current_value);
}

static void start_chirp(void) {
  chirp_reset(&chirp, get_sys_time_float());
  chirp_active = true;
  set_current_chirp_values();
}

static void stop_chirp(void) {
  chirp_reset(&chirp, get_sys_time_float());
  chirp_active = false;
  set_current_chirp_values();
}

void sys_id_chirp_chirp_activate_handler(uint8_t activate) {
  chirp_active = activate;
  if (chirp_active) {
    chirp_init(&chirp, chirp_f0_hz, chirp_f1_hz, chirp_length_s, get_sys_time_float(), CHIRP_EXPONENTIAL, CHIRP_FADEIN);
    start_chirp();
  }
  else
    stop_chirp();
}

void sys_id_chirp_init(void) {
  chirp_init(&chirp, chirp_f0_hz, chirp_f1_hz, chirp_length_s, get_sys_time_float(), CHIRP_EXPONENTIAL, CHIRP_FADEIN);
  set_current_chirp_values();
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_CHIRP, send_chirp);

  // Filter cutoff frequency is the chirp maximum frequency
  float tau = 1 / (chirp_f1_hz * 2 * M_PI);
  for (uint8_t i = 0; i < CHIRP_NO_AXES; i++) {
    init_first_order_low_pass(&filters[i], tau, SYS_ID_CHIRP_RUN_PERIOD, 0);
    current_chirp_values[i] = 0;
  }
}

void sys_id_chirp_run(void) {
  if (chirp_active) {
    if (!chirp_is_running(&chirp, get_sys_time_float()))
      stop_chirp();
    else {
      chirp_update(&chirp, get_sys_time_float());
      set_current_chirp_values();
    }
  }
}
