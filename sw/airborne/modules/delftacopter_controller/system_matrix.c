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
	{  0.998709,   0.000419,   0.003775,   0.223712,  -0.043267,   0.016833},
	{  0.001337,   0.995769,   1.069700,  -0.030737,   0.053037,  -0.127524},
	{  0.000097,  -0.001934,   0.957770,  -0.055624,   0.001678,  -0.000825},
	{ -0.001938,  -0.000027,   0.031254,   0.958598,  -0.000991,   0.000153},
	{  0.046885,   0.000253,   0.000150,   0.005354,   0.921310,   0.010135},
	{  0.000527,   0.045856,   0.025260,  -0.000450,   0.021271,   0.880426}
};
float _B_obsv_hover[OBSERVER_N_STATES][OBSERVER_N_INPUTS] = {
	{  0.002291,  -0.000400,   0.000000,   0.044338,  -0.017254},
	{  0.000617,   0.006823,   0.000000,  -0.054354,   0.130702},
	{  0.000929,   0.012428,   0.000000,  -0.001720,   0.000847},
	{  0.019921,  -0.003418,   0.000000,   0.001016,  -0.000157},
	{  0.000037,  -0.000006,   0.000000,   0.031801,  -0.010388},
	{  0.000011,   0.000108,   0.000000,  -0.021798,   0.073701}
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
	{  0.998232,   0.000553,   0.003824,   0.223675,  -0.062421,   0.022061},
	{  0.001930,   0.994560,   1.069258,  -0.030686,   0.076799,  -0.175653},
	{  0.000137,  -0.001968,   0.957758,  -0.055621,   0.003269,  -0.002133},
	{ -0.001969,  -0.000024,   0.031255,   0.958596,  -0.002215,   0.000269},
	{  0.046630,   0.000308,   0.000170,   0.005334,   0.911030,   0.012293},
	{  0.000645,   0.045419,   0.025100,  -0.000439,   0.026010,   0.862937}
};
float _B_obsv_hover[OBSERVER_N_STATES][OBSERVER_N_INPUTS] = {
	{  0.002291,  -0.000400,   0.000000,   0.063969,  -0.022616},
	{  0.000617,   0.006821,   0.000000,  -0.078709,   0.180041},
	{  0.000929,   0.012428,   0.000000,  -0.003351,   0.002188},
	{  0.019921,  -0.003418,   0.000000,   0.002271,  -0.000276},
	{  0.000037,  -0.000006,   0.000000,   0.042337,  -0.012601},
	{  0.000011,   0.000107,   0.000000,  -0.026655,   0.091627}
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
	{  0.997315,   0.000772,   0.003905,   0.223604,  -0.099148,   0.030552},
	{  0.002962,   0.992455,   1.068486,  -0.030595,   0.118049,  -0.258992},
	{  0.000228,  -0.002047,   0.957728,  -0.055613,   0.006896,  -0.005234},
	{ -0.002055,  -0.000013,   0.031259,   0.958589,  -0.005588,   0.000648},
	{  0.046206,   0.000380,   0.000197,   0.005302,   0.893962,   0.015111},
	{  0.000804,   0.044771,   0.024863,  -0.000423,   0.032354,   0.837097}
};
float _B_obsv_hover[OBSERVER_N_STATES][OBSERVER_N_INPUTS] = {
	{  0.002290,  -0.000400,   0.000000,   0.101613,  -0.031326},
	{  0.000618,   0.006819,   0.000000,  -0.120991,   0.265486},
	{  0.000929,   0.012428,   0.000000,  -0.007069,   0.005369},
	{  0.019921,  -0.003418,   0.000000,   0.005730,  -0.000665},
	{  0.000036,  -0.000006,   0.000000,   0.059829,  -0.015492},
	{  0.000011,   0.000107,   0.000000,  -0.033157,   0.118116}
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
	{  0.996334,   0.000972,   0.003979,   0.223528,  -0.138290,   0.038227},
	{  0.003949,   0.990385,   1.067725,  -0.030509,   0.157262,  -0.340478},
	{  0.000339,  -0.002147,   0.957691,  -0.055604,   0.011277,  -0.009101},
	{ -0.002178,   0.000002,   0.031265,   0.958579,  -0.010430,   0.001208},
	{  0.045815,   0.000433,   0.000217,   0.005272,   0.878271,   0.017114},
	{  0.000921,   0.044222,   0.024662,  -0.000411,   0.037006,   0.815333}
};
float _B_obsv_hover[OBSERVER_N_STATES][OBSERVER_N_INPUTS] = {
	{  0.002290,  -0.000400,   0.000000,   0.141735,  -0.039202},
	{  0.000618,   0.006816,   0.000000,  -0.161190,   0.349041},
	{  0.000929,   0.012428,   0.000000,  -0.011561,   0.009336},
	{  0.019921,  -0.003418,   0.000000,   0.010695,  -0.001241},
	{  0.000036,  -0.000006,   0.000000,   0.075911,  -0.017547},
	{  0.000011,   0.000106,   0.000000,  -0.037927,   0.140429}
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
	{  0.995030,   0.001205,   0.004065,   0.223427,  -0.190128,   0.047005},
	{  0.005123,   0.987798,   1.066772,  -0.030405,   0.203743,  -0.441815},
	{  0.000499,  -0.002297,   0.957636,  -0.055591,   0.017586,  -0.014866},
	{ -0.002384,   0.000027,   0.031274,   0.958563,  -0.018512,   0.002111},
	{  0.045360,   0.000482,   0.000235,   0.005237,   0.860075,   0.018936},
	{  0.001033,   0.043617,   0.024440,  -0.000400,   0.041395,   0.791445}
};
float _B_obsv_hover[OBSERVER_N_STATES][OBSERVER_N_INPUTS] = {
	{  0.002290,  -0.000399,   0.000000,   0.194878,  -0.048213},
	{  0.000618,   0.006813,   0.000000,  -0.208846,   0.452966},
	{  0.000929,   0.012428,   0.000000,  -0.018030,   0.015251},
	{  0.019921,  -0.003418,   0.000000,   0.018983,  -0.002169},
	{  0.000036,  -0.000006,   0.000000,   0.094562,  -0.019419},
	{  0.000011,   0.000105,   0.000000,  -0.042427,   0.164922}
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
	{  0.992663,   0.001564,   0.004199,   0.223244,  -0.283770,   0.060303},
	{  0.006982,   0.983365,   1.065133,  -0.030238,   0.276810,  -0.614197},
	{  0.000812,  -0.002608,   0.957520,  -0.055565,   0.029885,  -0.026716},
	{ -0.002863,   0.000081,   0.031295,   0.958526,  -0.037160,   0.004025},
	{  0.044654,   0.000540,   0.000257,   0.005182,   0.832005,   0.020999},
	{  0.001168,   0.042718,   0.024109,  -0.000384,   0.046650,   0.756203}
};
float _B_obsv_hover[OBSERVER_N_STATES][OBSERVER_N_INPUTS] = {
	{  0.002289,  -0.000399,   0.000000,   0.290887,  -0.061870},
	{  0.000618,   0.006808,   0.000000,  -0.283773,   0.629781},
	{  0.000930,   0.012427,   0.000000,  -0.030642,   0.027412},
	{  0.019921,  -0.003418,   0.000000,   0.038110,  -0.004138},
	{  0.000036,  -0.000006,   0.000000,   0.123337,  -0.021539},
	{  0.000011,   0.000104,   0.000000,  -0.047818,   0.201063}
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
	{  0.990244,   0.001876,   0.004317,   0.223056,  -0.378878,   0.071622},
	{  0.008634,   0.979036,   1.063525,  -0.030087,   0.341208,  -0.781235},
	{  0.001151,  -0.002967,   0.957386,  -0.055537,   0.043115,  -0.040297},
	{ -0.003468,   0.000144,   0.031319,   0.958479,  -0.060624,   0.006190},
	{  0.044038,   0.000577,   0.000271,   0.005135,   0.807587,   0.022265},
	{  0.001260,   0.041954,   0.023826,  -0.000373,   0.050151,   0.726443}
};
float _B_obsv_hover[OBSERVER_N_STATES][OBSERVER_N_INPUTS] = {
	{  0.002288,  -0.000398,   0.000000,   0.388414,  -0.073501},
	{  0.000618,   0.006802,   0.000000,  -0.349822,   0.801150},
	{  0.000930,   0.012427,   0.000000,  -0.044212,   0.041351},
	{  0.019920,  -0.003418,   0.000000,   0.062180,  -0.006364},
	{  0.000036,  -0.000006,   0.000000,   0.148372,  -0.022843},
	{  0.000011,   0.000103,   0.000000,  -0.051410,   0.231587}
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
	{  0.987133,   0.002224,   0.004449,   0.222814,  -0.500488,   0.083983},
	{  0.010510,   0.973636,   1.061513,  -0.029912,   0.413748,  -0.987893},
	{  0.001606,  -0.003481,   0.957192,  -0.055499,   0.060715,  -0.059579},
	{ -0.004396,   0.000230,   0.031352,   0.958406,  -0.096316,   0.009138},
	{  0.043344,   0.000609,   0.000283,   0.005081,   0.780279,   0.023267},
	{  0.001342,   0.041109,   0.023513,  -0.000362,   0.053219,   0.693727}
};
float _B_obsv_hover[OBSERVER_N_STATES][OBSERVER_N_INPUTS] = {
	{  0.002287,  -0.000397,   0.000000,   0.513135,  -0.086210},
	{  0.000618,   0.006796,   0.000000,  -0.424239,   1.013208},
	{  0.000930,   0.012426,   0.000000,  -0.062266,   0.061148},
	{  0.019920,  -0.003417,   0.000000,   0.098799,  -0.009400},
	{  0.000035,  -0.000005,   0.000000,   0.176373,  -0.023876},
	{  0.000011,   0.000102,   0.000000,  -0.054561,   0.265148}
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
	{  0.981660,   0.002743,   0.004649,   0.222387,  -0.712608,   0.101805},
	{  0.013359,   0.964411,   1.058058,  -0.029639,   0.522560,  -1.337139},
	{  0.002437,  -0.004509,   0.956803,  -0.055429,   0.092563,  -0.097619},
	{ -0.006375,   0.000394,   0.031416,   0.958251,  -0.171691,   0.014529},
	{  0.042302,   0.000642,   0.000296,   0.005001,   0.739493,   0.024182},
	{  0.001436,   0.039851,   0.023044,  -0.000348,   0.056606,   0.645525}
};
float _B_obsv_hover[OBSERVER_N_STATES][OBSERVER_N_INPUTS] = {
	{  0.002284,  -0.000396,   0.000000,   0.730729,  -0.104551},
	{  0.000618,   0.006784,   0.000000,  -0.535900,   1.371681},
	{  0.000930,   0.012425,   0.000000,  -0.094945,   0.100217},
	{  0.019919,  -0.003417,   0.000000,   0.176153,  -0.014955},
	{  0.000035,  -0.000005,   0.000000,   0.218203,  -0.024824},
	{  0.000010,   0.000101,   0.000000,  -0.058042,   0.314609}
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
	{  0.976214,   0.003181,   0.004819,   0.221961,  -0.921757,   0.116269},
	{  0.015812,   0.955437,   1.054678,  -0.029396,   0.614929,  -1.672657},
	{  0.003290,  -0.005668,   0.956362,  -0.055355,   0.124867,  -0.139869},
	{ -0.008722,   0.000566,   0.031484,   0.958066,  -0.260165,   0.019972},
	{  0.041409,   0.000661,   0.000304,   0.004932,   0.704886,   0.024568},
	{  0.001496,   0.038783,   0.022644,  -0.000337,   0.058672,   0.605028}
};
float _B_obsv_hover[OBSERVER_N_STATES][OBSERVER_N_INPUTS] = {
	{  0.002282,  -0.000395,   0.000000,   0.945324,  -0.119453},
	{  0.000618,   0.006773,   0.000000,  -0.630722,   1.716175},
	{  0.000930,   0.012424,   0.000000,  -0.128102,   0.143626},
	{  0.019918,  -0.003417,   0.000000,   0.266975,  -0.020569},
	{  0.000035,  -0.000005,   0.000000,   0.253701,  -0.025229},
	{  0.000010,   0.000099,   0.000000,  -0.060168,   0.356174}
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
	{  0.969344,   0.003658,   0.005007,   0.221421,  -1.183029,   0.131403},
	{  0.018540,   0.944308,   1.050462,  -0.029118,   0.716124,  -2.083540},
	{  0.004389,  -0.007298,   0.955737,  -0.055260,   0.166016,  -0.198395},
	{ -0.012151,   0.000791,   0.031574,   0.957794,  -0.387953,   0.026793},
	{  0.040421,   0.000674,   0.000310,   0.004855,   0.666866,   0.024697},
	{  0.001548,   0.037605,   0.022201,  -0.000326,   0.060323,   0.560853}
};
float _B_obsv_hover[OBSERVER_N_STATES][OBSERVER_N_INPUTS] = {
	{  0.002280,  -0.000394,   0.000000,   1.213466,  -0.135065},
	{  0.000618,   0.006759,   0.000000,  -0.734645,   2.138190},
	{  0.000930,   0.012421,   0.000000,  -0.170350,   0.203781},
	{  0.019917,  -0.003416,   0.000000,   0.398191,  -0.027615},
	{  0.000034,  -0.000005,   0.000000,   0.292710,  -0.025371},
	{  0.000010,   0.000098,   0.000000,  -0.061870,   0.401527}
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
	{  0.957511,   0.004350,   0.005285,   0.220487,  -1.627213,   0.152034},
	{  0.022595,   0.925462,   1.043266,  -0.028688,   0.863453,  -2.767118},
	{  0.006323,  -0.010488,   0.954503,  -0.055088,   0.237218,  -0.310557},
	{ -0.019122,   0.001191,   0.031737,   0.957238,  -0.643786,   0.038194},
	{  0.038955,   0.000684,   0.000316,   0.004740,   0.611103,   0.024485},
	{  0.001603,   0.035864,   0.021541,  -0.000311,   0.061887,   0.496594}
};
float _B_obsv_hover[OBSERVER_N_STATES][OBSERVER_N_INPUTS] = {
	{  0.002275,  -0.000392,   0.000000,   1.669483,  -0.156388},
	{  0.000617,   0.006735,   0.000000,  -0.886029,   2.840616},
	{  0.000931,   0.012417,   0.000000,  -0.243486,   0.319134},
	{  0.019914,  -0.003415,   0.000000,   0.660996,  -0.039416},
	{  0.000034,  -0.000005,   0.000000,   0.349939,  -0.025169},
	{  0.000010,   0.000096,   0.000000,  -0.063490,   0.467526}
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
	{ -0.010226,   0.007424,   0.099681,  -0.193451},
	{ -0.003796,  -0.007448,  -0.122274,   0.107771},
	{  0.000000,   0.000000,   0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.104277,   0.020175},
	{ -0.007670,   0.159246},
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
	{ -0.023667,   0.015776,   0.214733,  -0.401741},
	{ -0.008739,  -0.017180,  -0.282690,   0.231217},
	{  0.000000,   0.000000,   0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.117718,   0.011823},
	{ -0.002727,   0.168978},
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
	{ -0.042797,   0.026129,   0.358951,  -0.644822},
	{ -0.015956,  -0.031337,  -0.513541,   0.385388},
	{  0.000000,   0.000000,   0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.136848,   0.001470},
	{  0.004491,   0.183135},
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
	{ -0.073945,   0.040755,   0.560585,  -0.960342},
	{ -0.028062,  -0.055514,  -0.895329,   0.601848},
	{  0.000000,   0.000000,   0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.167996,  -0.013156},
	{  0.016597,   0.207312},
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
	{ -0.142267,   0.067146,   0.902452,  -1.462359},
	{ -0.054015,  -0.112767,  -1.731301,   0.979979},
	{  0.000000,   0.000000,   0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.236318,  -0.039547},
	{  0.042550,   0.264564},
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
	{ -0.223928,   0.091999,   1.184877,  -1.885151},
	{ -0.081353,  -0.185692,  -2.681058,   1.317525},
	{  0.000000,   0.000000,   0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.317980,  -0.064400},
	{  0.069887,   0.337490},
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
	{ -0.343463,   0.119713,   1.444044,  -2.339240},
	{ -0.113915,  -0.296144,  -3.944759,   1.675398},
	{  0.000000,   0.000000,   0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.437514,  -0.092114},
	{  0.102449,   0.447942},
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
	{ -0.586551,   0.156894,   1.679927,  -3.007730},
	{ -0.161322,  -0.524608,  -6.134308,   2.143632},
	{  0.000000,   0.000000,   0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.680602,  -0.129295},
	{  0.149856,   0.676405},
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
	{ -0.863009,   0.181266,   1.734148,  -3.596911},
	{ -0.197097,  -0.785003,  -8.204101,   2.475617},
	{  0.000000,   0.000000,   0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.957060,  -0.153667},
	{  0.185631,   0.936801},
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
	{  0.009030,   0.000549},
	{ -0.007703,  -0.012105},
	{  0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.085021,   0.027050},
	{ -0.003763,   0.163903},
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
	{  0.019685,   0.000289},
	{ -0.017165,  -0.026618},
	{  0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.074366,   0.027310},
	{  0.005699,   0.178416},
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
	{  0.023006,   0.008378},
	{ -0.002470,  -0.038020},
	{  0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.071045,   0.019221},
	{ -0.008996,   0.189818},
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
	{  0.044880,  -0.007362},
	{ -0.077172,  -0.073983},
	{  0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.049171,   0.034960},
	{  0.065706,   0.225781},
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
	{  0.057322,   0.001058},
	{ -0.035181,  -0.093905},
	{  0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.036729,   0.026540},
	{  0.023715,   0.245703},
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
	{ -0.016233,   0.033381},
	{ -0.142185,  -0.217474},
	{  0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.110284,  -0.005782},
	{  0.130719,   0.369272},
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
	{ -0.052817,   0.024731},
	{ -0.179386,  -0.239261},
	{  0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.146868,   0.002868},
	{  0.167921,   0.391059},
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
	{  0.057533,  -0.104341},
	{  0.050676,  -0.472351},
	{  0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.036518,   0.131939},
	{ -0.062141,   0.624149},
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
	{ -0.268723,   0.044099},
	{ -0.472228,  -0.533580},
	{  0.000000,   0.000000}
};
float _controller_g[SYSTEM_N_INPUTS][SYSTEM_N_OUTPUTS] = {
	{  0.362774,  -0.016501},
	{  0.460762,   0.685378},
	{  0.000000,   0.000000}
};
#endif

#ifndef OBSERVER_MATRIX_PICKED
#error "Observer matrix was not selected"
#endif

#ifndef CONTROLLER_MATRIX_PICKED
#error "Controller matrix was not selected"
#endif
