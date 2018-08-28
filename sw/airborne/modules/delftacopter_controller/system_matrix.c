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
 * @file system_matrix.c
 * @brief Here the Delftacopter observer and controller matrices are defined
 * This is mainly done to have a single place for choosing which matrix to use
 * i.e. for the DC3 different matrices are required.
 */

#include "system_matrix.h"

#ifdef observer_DC3_6S6P_2800RPM_0_0001
#define OBSERVER_MATRIX_PICKED
uint16_t observer_matrix_id = 0;
// 0.00010
float _A_obsv_hover[OBSERVER_N_STATES][OBSERVER_N_STATES] = {
	{  0.998584,   0.000484,   0.004643,   0.275256,  -0.046209,   0.019451},
	{  0.001557,   0.995881,   0.938816,  -0.032656,   0.061820,  -0.128215},
	{  0.000122,  -0.001943,   0.957979,  -0.067423,   0.002188,  -0.001158},
	{ -0.001932,  -0.000031,   0.031474,   0.958631,  -0.000739,   0.000002},
	{  0.046851,   0.000292,   0.000165,   0.006585,   0.919978,   0.011703},
	{  0.000609,   0.045859,   0.022169,  -0.000465,   0.024573,   0.880480}
};
float _B_obsv_hover[OBSERVER_N_STATES][OBSERVER_N_INPUTS] = {
	{  0.002736,  -0.000662,   0.000000,   0.047355,  -0.019939},
	{  0.000095,   0.006099,   0.000000,  -0.063356,   0.131410},
	{ -0.000030,   0.012682,   0.000000,  -0.002243,   0.001188},
	{  0.019332,  -0.004613,   0.000000,   0.000759,  -0.000002},
	{  0.000044,  -0.000010,   0.000000,   0.033166,  -0.011995},
	{  0.000003,   0.000096,   0.000000,  -0.025182,   0.073646}
};
float _C_obsv_hover[OBSERVER_N_OUTPUTS][OBSERVER_N_STATES] = {
	{  1.000000,   0.000000,   0.000000,   0.000000,   0.000000,   0.000000},
	{  0.000000,   1.000000,   0.000000,   0.000000,   0.000000,   0.000000},
	{  0.000000,   0.000000,   1.000000,   0.000000,   0.000000,   0.000000},
	{  0.000000,   0.000000,   0.000000,   1.000000,   0.000000,   0.000000}
};
#endif

#ifdef observer_DC3_6S6P_2800RPM_0_0002
#define OBSERVER_MATRIX_PICKED
uint16_t observer_matrix_id = 1;
// 0.00020
float _A_obsv_hover[OBSERVER_N_STATES][OBSERVER_N_STATES] = {
	{  0.998060,   0.000645,   0.004694,   0.275206,  -0.067254,   0.025721},
	{  0.002236,   0.994702,   0.938438,  -0.032585,   0.089016,  -0.175103},
	{  0.000172,  -0.001982,   0.957966,  -0.067418,   0.004205,  -0.002694},
	{ -0.001959,  -0.000028,   0.031475,   0.958629,  -0.001810,   0.000097},
	{  0.046580,   0.000356,   0.000186,   0.006559,   0.909049,   0.014209},
	{  0.000745,   0.045436,   0.022033,  -0.000449,   0.030059,   0.863553}
};
float _B_obsv_hover[OBSERVER_N_STATES][OBSERVER_N_INPUTS] = {
	{  0.002735,  -0.000662,   0.000000,   0.068923,  -0.026369},
	{  0.000095,   0.006098,   0.000000,  -0.091231,   0.179477},
	{ -0.000030,   0.012682,   0.000000,  -0.004311,   0.002763},
	{  0.019332,  -0.004613,   0.000000,   0.001856,  -0.000100},
	{  0.000044,  -0.000010,   0.000000,   0.044367,  -0.014565},
	{  0.000003,   0.000096,   0.000000,  -0.030804,   0.090996}
};
float _C_obsv_hover[OBSERVER_N_OUTPUTS][OBSERVER_N_STATES] = {
	{  1.000000,   0.000000,   0.000000,   0.000000,   0.000000,   0.000000},
	{  0.000000,   1.000000,   0.000000,   0.000000,   0.000000,   0.000000},
	{  0.000000,   0.000000,   1.000000,   0.000000,   0.000000,   0.000000},
	{  0.000000,   0.000000,   0.000000,   1.000000,   0.000000,   0.000000}
};
#endif

#ifdef observer_DC3_6S6P_2800RPM_0_0005
#define OBSERVER_MATRIX_PICKED
uint16_t observer_matrix_id = 2;
// 0.00050
float _A_obsv_hover[OBSERVER_N_STATES][OBSERVER_N_STATES] = {
	{  0.997045,   0.000913,   0.004781,   0.275109,  -0.107853,   0.036140},
	{  0.003421,   0.992668,   0.937784,  -0.032462,   0.136270,  -0.255621},
	{  0.000288,  -0.002073,   0.957936,  -0.067407,   0.008790,  -0.006250},
	{ -0.002037,  -0.000017,   0.031479,   0.958621,  -0.004880,   0.000498},
	{  0.046128,   0.000441,   0.000213,   0.006516,   0.890879,   0.017531},
	{  0.000932,   0.044811,   0.021833,  -0.000428,   0.037486,   0.838641}
};
float _B_obsv_hover[OBSERVER_N_STATES][OBSERVER_N_INPUTS] = {
	{  0.002735,  -0.000661,   0.000000,   0.110537,  -0.037057},
	{  0.000096,   0.006095,   0.000000,  -0.139670,   0.262029},
	{ -0.000030,   0.012682,   0.000000,  -0.009011,   0.006411},
	{  0.019332,  -0.004613,   0.000000,   0.005004,  -0.000512},
	{  0.000043,  -0.000010,   0.000000,   0.062988,  -0.017973},
	{  0.000003,   0.000095,   0.000000,  -0.038417,   0.116533}
};
float _C_obsv_hover[OBSERVER_N_OUTPUTS][OBSERVER_N_STATES] = {
	{  1.000000,   0.000000,   0.000000,   0.000000,   0.000000,   0.000000},
	{  0.000000,   1.000000,   0.000000,   0.000000,   0.000000,   0.000000},
	{  0.000000,   0.000000,   1.000000,   0.000000,   0.000000,   0.000000},
	{  0.000000,   0.000000,   0.000000,   1.000000,   0.000000,   0.000000}
};
#endif

#ifdef observer_DC3_6S6P_2800RPM_0_001
#define OBSERVER_MATRIX_PICKED
uint16_t observer_matrix_id = 3;
// 0.00100
float _A_obsv_hover[OBSERVER_N_STATES][OBSERVER_N_STATES] = {
	{  0.995955,   0.001165,   0.004862,   0.275005,  -0.151289,   0.045781},
	{  0.004555,   0.990684,   0.937144,  -0.032344,   0.181303,  -0.333780},
	{  0.000428,  -0.002186,   0.957900,  -0.067393,   0.014320,  -0.010609},
	{ -0.002153,   0.000001,   0.031484,   0.958610,  -0.009388,   0.001150},
	{  0.045711,   0.000505,   0.000234,   0.006477,   0.874180,   0.019938},
	{  0.001071,   0.044283,   0.021663,  -0.000412,   0.043009,   0.817713}
};
float _B_obsv_hover[OBSERVER_N_STATES][OBSERVER_N_INPUTS] = {
	{  0.002735,  -0.000661,   0.000000,   0.155064,  -0.046949},
	{  0.000096,   0.006093,   0.000000,  -0.185837,   0.342173},
	{ -0.000030,   0.012682,   0.000000,  -0.014681,   0.010883},
	{  0.019332,  -0.004613,   0.000000,   0.009627,  -0.001182},
	{  0.000043,  -0.000010,   0.000000,   0.080105,  -0.020443},
	{  0.000003,   0.000095,   0.000000,  -0.044080,   0.137989}
};
float _C_obsv_hover[OBSERVER_N_OUTPUTS][OBSERVER_N_STATES] = {
	{  1.000000,   0.000000,   0.000000,   0.000000,   0.000000,   0.000000},
	{  0.000000,   1.000000,   0.000000,   0.000000,   0.000000,   0.000000},
	{  0.000000,   0.000000,   1.000000,   0.000000,   0.000000,   0.000000},
	{  0.000000,   0.000000,   0.000000,   1.000000,   0.000000,   0.000000}
};
#endif

#ifdef observer_DC3_6S6P_2800RPM_0_002
#define OBSERVER_MATRIX_PICKED
uint16_t observer_matrix_id = 4;
// 0.00200
float _A_obsv_hover[OBSERVER_N_STATES][OBSERVER_N_STATES] = {
	{  0.994502,   0.001463,   0.004959,   0.274867,  -0.208935,   0.057051},
	{  0.005912,   0.988216,   0.936347,  -0.032202,   0.234881,  -0.430455},
	{  0.000630,  -0.002353,   0.957845,  -0.067373,   0.022283,  -0.017026},
	{ -0.002348,   0.000031,   0.031495,   0.958591,  -0.017004,   0.002256},
	{  0.045227,   0.000565,   0.000253,   0.006431,   0.854833,   0.022179},
	{  0.001206,   0.043702,   0.021476,  -0.000396,   0.048302,   0.794774}
};
float _B_obsv_hover[OBSERVER_N_STATES][OBSERVER_N_INPUTS] = {
	{  0.002734,  -0.000660,   0.000000,   0.214162,  -0.058516},
	{  0.000097,   0.006090,   0.000000,  -0.240772,   0.441317},
	{ -0.000029,   0.012681,   0.000000,  -0.022847,   0.017466},
	{  0.019331,  -0.004613,   0.000000,   0.017439,  -0.002318},
	{  0.000043,  -0.000010,   0.000000,   0.099936,  -0.022744},
	{  0.000003,   0.000094,   0.000000,  -0.049507,   0.161510}
};
float _C_obsv_hover[OBSERVER_N_OUTPUTS][OBSERVER_N_STATES] = {
	{  1.000000,   0.000000,   0.000000,   0.000000,   0.000000,   0.000000},
	{  0.000000,   1.000000,   0.000000,   0.000000,   0.000000,   0.000000},
	{  0.000000,   0.000000,   1.000000,   0.000000,   0.000000,   0.000000},
	{  0.000000,   0.000000,   0.000000,   1.000000,   0.000000,   0.000000}
};
#endif

#ifdef observer_DC3_6S6P_2800RPM_0_005
#define OBSERVER_MATRIX_PICKED
uint16_t observer_matrix_id = 5;
// 0.00500
float _A_obsv_hover[OBSERVER_N_STATES][OBSERVER_N_STATES] = {
	{  0.991860,   0.001934,   0.005113,   0.274614,  -0.313207,   0.074615},
	{  0.008075,   0.984010,   0.934983,  -0.031973,   0.319642,  -0.594054},
	{  0.001027,  -0.002694,   0.957734,  -0.067333,   0.037823,  -0.030045},
	{ -0.002804,   0.000100,   0.031517,   0.958548,  -0.034737,   0.004724},
	{  0.044477,   0.000637,   0.000277,   0.006360,   0.825033,   0.024812},
	{  0.001374,   0.042841,   0.021197,  -0.000375,   0.054795,   0.760950}
};
float _B_obsv_hover[OBSERVER_N_STATES][OBSERVER_N_INPUTS] = {
	{  0.002733,  -0.000660,   0.000000,   0.321076,  -0.076553},
	{  0.000098,   0.006086,   0.000000,  -0.327696,   0.609122},
	{ -0.000029,   0.012681,   0.000000,  -0.038784,   0.030827},
	{  0.019331,  -0.004612,   0.000000,   0.035628,  -0.004855},
	{  0.000043,  -0.000010,   0.000000,   0.130486,  -0.025450},
	{  0.000003,   0.000093,   0.000000,  -0.056169,   0.196195}
};
float _C_obsv_hover[OBSERVER_N_OUTPUTS][OBSERVER_N_STATES] = {
	{  1.000000,   0.000000,   0.000000,   0.000000,   0.000000,   0.000000},
	{  0.000000,   1.000000,   0.000000,   0.000000,   0.000000,   0.000000},
	{  0.000000,   0.000000,   1.000000,   0.000000,   0.000000,   0.000000},
	{  0.000000,   0.000000,   0.000000,   1.000000,   0.000000,   0.000000}
};
#endif

#ifdef observer_DC3_6S6P_2800RPM_0_01
#define OBSERVER_MATRIX_PICKED
uint16_t observer_matrix_id = 6;
// 0.01000
float _A_obsv_hover[OBSERVER_N_STATES][OBSERVER_N_STATES] = {
	{  0.989158,   0.002356,   0.005252,   0.274356,  -0.419160,   0.090051},
	{  0.010014,   0.979920,   0.933651,  -0.031764,   0.394961,  -0.751951},
	{  0.001459,  -0.003084,   0.957606,  -0.067290,   0.054574,  -0.044808},
	{ -0.003386,   0.000183,   0.031545,   0.958492,  -0.057160,   0.007649},
	{  0.043822,   0.000687,   0.000294,   0.006298,   0.799153,   0.026525},
	{  0.001491,   0.042108,   0.020960,  -0.000360,   0.059265,   0.732383}
};
float _B_obsv_hover[OBSERVER_N_STATES][OBSERVER_N_INPUTS] = {
	{  0.002731,  -0.000659,   0.000000,   0.429732,  -0.092411},
	{  0.000098,   0.006081,   0.000000,  -0.404954,   0.771110},
	{ -0.000029,   0.012680,   0.000000,  -0.055966,   0.045979},
	{  0.019331,  -0.004612,   0.000000,   0.058633,  -0.007863},
	{  0.000042,  -0.000010,   0.000000,   0.157021,  -0.027212},
	{  0.000003,   0.000092,   0.000000,  -0.060756,   0.225495}
};
float _C_obsv_hover[OBSERVER_N_OUTPUTS][OBSERVER_N_STATES] = {
	{  1.000000,   0.000000,   0.000000,   0.000000,   0.000000,   0.000000},
	{  0.000000,   1.000000,   0.000000,   0.000000,   0.000000,   0.000000},
	{  0.000000,   0.000000,   1.000000,   0.000000,   0.000000,   0.000000},
	{  0.000000,   0.000000,   0.000000,   1.000000,   0.000000,   0.000000}
};
#endif

#ifdef observer_DC3_6S6P_2800RPM_0_02
#define OBSERVER_MATRIX_PICKED
uint16_t observer_matrix_id = 7;
// 0.02000
float _A_obsv_hover[OBSERVER_N_STATES][OBSERVER_N_STATES] = {
	{  0.985681,   0.002842,   0.005413,   0.274022,  -0.554619,   0.107450},
	{  0.012239,   0.974835,   0.931990,  -0.031522,   0.480552,  -0.946790},
	{  0.002039,  -0.003638,   0.957424,  -0.067231,   0.076912,  -0.065601},
	{ -0.004279,   0.000303,   0.031585,   0.958405,  -0.091347,   0.011810},
	{  0.043087,   0.000731,   0.000308,   0.006228,   0.770258,   0.027986},
	{  0.001600,   0.041297,   0.020696,  -0.000344,   0.063333,   0.700958}
};
float _B_obsv_hover[OBSERVER_N_STATES][OBSERVER_N_INPUTS] = {
	{  0.002730,  -0.000658,   0.000000,   0.568668,  -0.110295},
	{  0.000099,   0.006075,   0.000000,  -0.492770,   0.971035},
	{ -0.000029,   0.012680,   0.000000,  -0.078884,   0.067327},
	{  0.019331,  -0.004612,   0.000000,   0.093713,  -0.012144},
	{  0.000042,  -0.000009,   0.000000,   0.186651,  -0.028717},
	{  0.000003,   0.000091,   0.000000,  -0.064933,   0.257731}
};
float _C_obsv_hover[OBSERVER_N_OUTPUTS][OBSERVER_N_STATES] = {
	{  1.000000,   0.000000,   0.000000,   0.000000,   0.000000,   0.000000},
	{  0.000000,   1.000000,   0.000000,   0.000000,   0.000000,   0.000000},
	{  0.000000,   0.000000,   1.000000,   0.000000,   0.000000,   0.000000},
	{  0.000000,   0.000000,   0.000000,   1.000000,   0.000000,   0.000000}
};
#endif

#ifdef observer_DC3_6S6P_2800RPM_0_05
#define OBSERVER_MATRIX_PICKED
uint16_t observer_matrix_id = 8;
// 0.05000
float _A_obsv_hover[OBSERVER_N_STATES][OBSERVER_N_STATES] = {
	{  0.979566,   0.003592,   0.005663,   0.273434,  -0.790730,   0.133628},
	{  0.015662,   0.966168,   0.929145,  -0.031142,   0.610504,  -1.275357},
	{  0.003104,  -0.004737,   0.957060,  -0.067122,   0.117487,  -0.106301},
	{ -0.006186,   0.000540,   0.031665,   0.958220,  -0.163617,   0.019856},
	{  0.041982,   0.000782,   0.000326,   0.006123,   0.727186,   0.029526},
	{  0.001732,   0.040089,   0.020302,  -0.000324,   0.068088,   0.654604}
};
float _B_obsv_hover[OBSERVER_N_STATES][OBSERVER_N_INPUTS] = {
	{  0.002727,  -0.000656,   0.000000,   0.810894,  -0.137223},
	{  0.000101,   0.006066,   0.000000,  -0.626145,   1.308270},
	{ -0.000028,   0.012678,   0.000000,  -0.120524,   0.109126},
	{  0.019330,  -0.004612,   0.000000,   0.167890,  -0.020426},
	{  0.000042,  -0.000009,   0.000000,   0.230827,  -0.030309},
	{  0.000003,   0.000090,   0.000000,  -0.069820,   0.305293}
};
float _C_obsv_hover[OBSERVER_N_OUTPUTS][OBSERVER_N_STATES] = {
	{  1.000000,   0.000000,   0.000000,   0.000000,   0.000000,   0.000000},
	{  0.000000,   1.000000,   0.000000,   0.000000,   0.000000,   0.000000},
	{  0.000000,   0.000000,   1.000000,   0.000000,   0.000000,   0.000000},
	{  0.000000,   0.000000,   0.000000,   1.000000,   0.000000,   0.000000}
};
#endif

#ifdef observer_DC3_6S6P_2800RPM_0_1
#define OBSERVER_MATRIX_PICKED
uint16_t observer_matrix_id = 9;
// 0.10000
float _A_obsv_hover[OBSERVER_N_STATES][OBSERVER_N_STATES] = {
	{  0.973483,   0.004251,   0.005886,   0.272847,  -1.023263,   0.155937},
	{  0.018654,   0.957754,   0.926367,  -0.030803,   0.722330,  -1.590603},
	{  0.004204,  -0.005966,   0.956649,  -0.067009,   0.158823,  -0.151228},
	{ -0.008450,   0.000801,   0.031754,   0.957999,  -0.248421,   0.028475},
	{  0.041039,   0.000815,   0.000337,   0.006032,   0.690716,   0.030401},
	{  0.001823,   0.039063,   0.019965,  -0.000309,   0.071223,   0.615602}
};
float _B_obsv_hover[OBSERVER_N_STATES][OBSERVER_N_INPUTS] = {
	{  0.002724,  -0.000655,   0.000000,   1.049511,  -0.160191},
	{  0.000102,   0.006056,   0.000000,  -0.740964,   1.631931},
	{ -0.000028,   0.012677,   0.000000,  -0.162960,   0.155283},
	{  0.019328,  -0.004611,   0.000000,   0.254959,  -0.029307},
	{  0.000041,  -0.000009,   0.000000,   0.268241,  -0.031216},
	{  0.000003,   0.000089,   0.000000,  -0.073046,   0.345322}
};
float _C_obsv_hover[OBSERVER_N_OUTPUTS][OBSERVER_N_STATES] = {
	{  1.000000,   0.000000,   0.000000,   0.000000,   0.000000,   0.000000},
	{  0.000000,   1.000000,   0.000000,   0.000000,   0.000000,   0.000000},
	{  0.000000,   0.000000,   1.000000,   0.000000,   0.000000,   0.000000},
	{  0.000000,   0.000000,   0.000000,   1.000000,   0.000000,   0.000000}
};
#endif

#ifdef observer_DC3_6S6P_2800RPM_0_2
#define OBSERVER_MATRIX_PICKED
uint16_t observer_matrix_id = 10;
// 0.20000
float _A_obsv_hover[OBSERVER_N_STATES][OBSERVER_N_STATES] = {
	{  0.965816,   0.004999,   0.006141,   0.272103,  -1.313340,   0.180451},
	{  0.022030,   0.947330,   0.922907,  -0.030411,   0.846454,  -1.976475},
	{  0.005629,  -0.007687,   0.956072,  -0.066860,   0.211703,  -0.213199},
	{ -0.011756,   0.001158,   0.031878,   0.957675,  -0.370771,   0.039921},
	{  0.039995,   0.000843,   0.000348,   0.005932,   0.650725,   0.031028},
	{  0.001907,   0.037929,   0.019591,  -0.000294,   0.073969,   0.572992}
};
float _B_obsv_hover[OBSERVER_N_STATES][OBSERVER_N_INPUTS] = {
	{  0.002720,  -0.000653,   0.000000,   1.347255,  -0.185453},
	{  0.000103,   0.006044,   0.000000,  -0.868464,   2.028229},
	{ -0.000027,   0.012675,   0.000000,  -0.217265,   0.218975},
	{  0.019327,  -0.004610,   0.000000,   0.380614,  -0.041110},
	{  0.000041,  -0.000009,   0.000000,   0.309276,  -0.031871},
	{  0.000003,   0.000088,   0.000000,  -0.075875,   0.389066}
};
float _C_obsv_hover[OBSERVER_N_OUTPUTS][OBSERVER_N_STATES] = {
	{  1.000000,   0.000000,   0.000000,   0.000000,   0.000000,   0.000000},
	{  0.000000,   1.000000,   0.000000,   0.000000,   0.000000,   0.000000},
	{  0.000000,   0.000000,   1.000000,   0.000000,   0.000000,   0.000000},
	{  0.000000,   0.000000,   0.000000,   1.000000,   0.000000,   0.000000}
};
#endif

#ifdef observer_DC3_6S6P_2800RPM_0_5
#define OBSERVER_MATRIX_PICKED
uint16_t observer_matrix_id = 11;
// 0.50000
float _A_obsv_hover[OBSERVER_N_STATES][OBSERVER_N_STATES] = {
	{  0.952622,   0.006140,   0.006536,   0.270816,  -1.805433,   0.216187},
	{  0.027146,   0.929692,   0.917007,  -0.029800,   1.030166,  -2.618543},
	{  0.008156,  -0.011040,   0.954936,  -0.066591,   0.303697,  -0.331531},
	{ -0.018472,   0.001829,   0.032113,   0.957012,  -0.615169,   0.060573},
	{  0.038449,   0.000874,   0.000360,   0.005783,   0.592218,   0.031483},
	{  0.002008,   0.036253,   0.019034,  -0.000273,   0.077005,   0.510879}
};
float _B_obsv_hover[OBSERVER_N_STATES][OBSERVER_N_INPUTS] = {
	{  0.002714,  -0.000650,   0.000000,   1.852542,  -0.222331},
	{  0.000105,   0.006024,   0.000000,  -1.057292,   2.687938},
	{ -0.000026,   0.012671,   0.000000,  -0.311787,   0.340660},
	{  0.019324,  -0.004608,   0.000000,   0.631729,  -0.062434},
	{  0.000040,  -0.000009,   0.000000,   0.369329,  -0.032357},
	{  0.000003,   0.000086,   0.000000,  -0.079013,   0.452855}
};
float _C_obsv_hover[OBSERVER_N_OUTPUTS][OBSERVER_N_STATES] = {
	{  1.000000,   0.000000,   0.000000,   0.000000,   0.000000,   0.000000},
	{  0.000000,   1.000000,   0.000000,   0.000000,   0.000000,   0.000000},
	{  0.000000,   0.000000,   1.000000,   0.000000,   0.000000,   0.000000},
	{  0.000000,   0.000000,   0.000000,   1.000000,   0.000000,   0.000000}
};
#endif

#ifdef controller_DC3_6S6P_2800RPM_lqr_500
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_OUTPUT_FEEDBACK
#error "Controller state size should be DC_STATE_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 0;
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.009633,   0.009741,   0.093758,  -0.217839},
	{ -0.004724,  -0.008396,  -0.110132,   0.119960},
	{  0.000000,   0.000000,   0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.107383,   0.027876},
	{ -0.000313,   0.159288},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC3_6S6P_2800RPM_lqr_200
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_OUTPUT_FEEDBACK
#error "Controller state size should be DC_STATE_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 1;
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.022385,   0.020197,   0.198004,  -0.444463},
	{ -0.010343,  -0.018747,  -0.249535,   0.250414},
	{  0.000000,   0.000000,   0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.120135,   0.017420},
	{  0.005306,   0.169640},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC3_6S6P_2800RPM_lqr_100
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_OUTPUT_FEEDBACK
#error "Controller state size should be DC_STATE_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 2;
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.040696,   0.032848,   0.326776,  -0.705735},
	{ -0.018226,  -0.033253,  -0.446850,   0.408243},
	{  0.000000,   0.000000,   0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.138446,   0.004769},
	{  0.013189,   0.184145},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC3_6S6P_2800RPM_lqr_50
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_OUTPUT_FEEDBACK
#error "Controller state size should be DC_STATE_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 3;
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.070762,   0.050541,   0.507160,  -1.045246},
	{ -0.031376,  -0.057379,  -0.771600,   0.625220},
	{  0.000000,   0.000000,   0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.168512,  -0.012925},
	{  0.026339,   0.208272},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC3_6S6P_2800RPM_lqr_20
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_OUTPUT_FEEDBACK
#error "Controller state size should be DC_STATE_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 4;
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.137149,   0.082758,   0.820879,  -1.593148},
	{ -0.060448,  -0.113489,  -1.487597,   0.999711},
	{  0.000000,   0.000000,   0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.234899,  -0.045142},
	{  0.055411,   0.264381},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC3_6S6P_2800RPM_lqr_10
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_OUTPUT_FEEDBACK
#error "Controller state size should be DC_STATE_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 5;
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.216728,   0.114121,   1.092769,  -2.063716},
	{ -0.092778,  -0.184553,  -2.313191,   1.333816},
	{  0.000000,   0.000000,   0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.314478,  -0.076504},
	{  0.087741,   0.335446},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC3_6S6P_2800RPM_lqr_5
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_OUTPUT_FEEDBACK
#error "Controller state size should be DC_STATE_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 6;
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.333484,   0.151014,   1.358596,  -2.576360},
	{ -0.133810,  -0.292401,  -3.429914,   1.689780},
	{  0.000000,   0.000000,   0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.431234,  -0.113397},
	{  0.128773,   0.443294},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC3_6S6P_2800RPM_lqr_2
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_OUTPUT_FEEDBACK
#error "Controller state size should be DC_STATE_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 7;
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.571975,   0.205793,   1.632120,  -3.336272},
	{ -0.199485,  -0.516955,  -5.403266,   2.157471},
	{  0.000000,   0.000000,   0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.669725,  -0.168176},
	{  0.194448,   0.667848},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC3_6S6P_2800RPM_lqr_1
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_OUTPUT_FEEDBACK
#error "Controller state size should be DC_STATE_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 8;
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.844563,   0.248508,   1.730964,  -4.004542},
	{ -0.255742,  -0.774841,  -7.300902,   2.487339},
	{  0.000000,   0.000000,   0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.942313,  -0.210891},
	{  0.250706,   0.925733},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC3_6S6P_2800RPM_lqr_output_500
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_STATE_FEEDBACK
#error "Controller state size should be DC_OUTPUT_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 9;
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{  0.010837,  -0.000081},
	{ -0.008834,  -0.016274},
	{  0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.086913,   0.037698},
	{  0.003797,   0.167166},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC3_6S6P_2800RPM_lqr_output_200
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_STATE_FEEDBACK
#error "Controller state size should be DC_OUTPUT_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 10;
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{  0.022539,  -0.001318},
	{ -0.019000,  -0.034926},
	{  0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.075211,   0.038935},
	{  0.013964,   0.185819},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC3_6S6P_2800RPM_lqr_output_100
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_STATE_FEEDBACK
#error "Controller state size should be DC_OUTPUT_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 11;
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{  0.031888,  -0.005829},
	{ -0.050782,  -0.058671},
	{  0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.065862,   0.043446},
	{  0.045746,   0.209564},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC3_6S6P_2800RPM_lqr_output_50
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_STATE_FEEDBACK
#error "Controller state size should be DC_OUTPUT_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 12;
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{  0.060998,   0.001822},
	{  0.031959,  -0.086192},
	{  0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.036752,   0.035795},
	{ -0.036996,   0.237084},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC3_6S6P_2800RPM_lqr_output_20
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_STATE_FEEDBACK
#error "Controller state size should be DC_OUTPUT_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 13;
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{  0.008545,  -0.043523},
	{ -0.307436,  -0.153266},
	{  0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.089205,   0.081140},
	{  0.302400,   0.304159},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC3_6S6P_2800RPM_lqr_output_10
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_STATE_FEEDBACK
#error "Controller state size should be DC_OUTPUT_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 14;
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.014724,   0.005684},
	{ -0.124994,  -0.301817},
	{  0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.112474,   0.031933},
	{  0.119957,   0.452710},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC3_6S6P_2800RPM_lqr_output_5
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_STATE_FEEDBACK
#error "Controller state size should be DC_OUTPUT_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 15;
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.054964,   0.032707},
	{ -0.190533,  -0.371047},
	{  0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.152714,   0.004910},
	{  0.185496,   0.521940},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC3_6S6P_2800RPM_lqr_output_2
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_STATE_FEEDBACK
#error "Controller state size should be DC_OUTPUT_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 16;
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.110709,  -0.042410},
	{ -0.215766,  -0.501149},
	{  0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.208459,   0.080027},
	{  0.210729,   0.652041},
	{  0.000000,   0.000000}
};
#endif

#ifdef controller_DC3_6S6P_2800RPM_lqr_output_1
#define CONTROLLER_MATRIX_PICKED
#ifdef DC_STATE_FEEDBACK
#error "Controller state size should be DC_OUTPUT_FEEDBACK in delftacopter_controller.xml"
#endif

uint16_t controller_matrix_id = 17;
float _controller_K[SYSTEM_N_INPUTS][CONTROLLER_N_STATES] = {
	{ -0.191032,   0.169716},
	{ -0.413449,  -0.528440},
	{  0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.288782,  -0.132099},
	{  0.408412,   0.679332},
	{  0.000000,   0.000000}
};
#endif

#ifndef OBSERVER_MATRIX_PICKED
#error "Observer matrix was not selected"
#endif

#ifndef CONTROLLER_MATRIX_PICKED
#error "Controller matrix was not selected"
#endif
