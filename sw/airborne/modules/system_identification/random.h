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
#ifndef RANDOM_H
#define RANDOM_H

#include <std.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include "generated/airframe.h"
#include "mcu_periph/sys_time.h"

// Random number from uniform[0,1] distribution
double rand_uniform(void);

// Random number from gaussian(0, 1) distribution
double rand_gaussian(void);

#endif // RANDOM_H
