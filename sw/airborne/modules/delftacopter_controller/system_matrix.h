/*
 * Copyright (C) 2018 Joost Meulenbeld
 *
 * This file is part of paparazzi.
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
 * @file system_matrix.h
 * @brief Here the Delftacopter observer and controller matrices are defined
 * This is mainly done to have a single place for choosing which matrix to use
 * i.e. for the DC3 different matrices are required.
 */

#ifndef SYSTEM_MATRIX_H
#define SYSTEM_MATRIX_H

#include <stdint.h>

/** 
 * System: Delftacopter inputs [roll, pitch, elevator] 
 *                      outputs [p, q]
 * Observer:            inputs [roll, pitch, elevator inputs, p_measured, q_measured]
 *                      states [p, q, a, b]
 *                      outputs [p, q, a, b] (all states)
 */
#define SYSTEM_N_INPUTS 3
#define SYSTEM_N_OUTPUTS 2
#define OBSERVER_N_STATES 6
#define OBSERVER_N_OUTPUTS 4
#ifdef DC_STATE_FEEDBACK
#define CONTROLLER_N_STATES OBSERVER_N_STATES
#endif
#ifdef DC_OUTPUT_FEEDBACK
#define CONTROLLER_N_STATES SYSTEM_N_OUTPUTS
#endif
#define OBSERVER_N_INPUTS SYSTEM_N_INPUTS + SYSTEM_N_OUTPUTS

/** Make a pointer to a matrix of _rows lines
 * @param _ptr The pointer which will be pointing
 * @param _mat The matrix that is allocated on the stack
 * @param _rows Number of rows of the matrix
*/
#define INIT_MATRIX_PTR(_ptr, _mat, _rows) \
    for (int __i = 0; __i < _rows; __i++) { _ptr[__i] = &_mat[__i][0]; }

extern uint16_t controller_matrix_id;
extern uint16_t observer_matrix_id;

extern float* A_obsv_hover[OBSERVER_N_STATES];
extern float* B_obsv_hover[OBSERVER_N_STATES];
extern float* C_obsv_hover[OBSERVER_N_OUTPUTS];
extern float* controller_K[SYSTEM_N_INPUTS];
extern float* controller_g[SYSTEM_N_INPUTS];

extern uint16_t current_observer_setting;
extern uint16_t current_controller_setting;

void update_observer_matrices(uint16_t new_type);
void update_controller_matrices(uint16_t new_type);
void init_system_matrices(void);

#endif // SYSTEM_MATRIX_H
