#include <math.h>
#include "inverter.h"

const double Vi[8][2] = { { 0.0, 0.0 },
		{ -1.0 / 3.0 * VDC, -SQRT3 / 3.0 * VDC }, { -1.0 / 3.0 * VDC, SQRT3
				/ 3.0 * VDC }, { -2.0 / 3.0 * VDC, 0.0 },
		{ 2.0 / 3.0 * VDC, 0.0 }, { 1.0 / 3.0 * VDC, -SQRT3 / 3.0 * VDC }, { 1.0
				/ 3.0 * VDC, SQRT3 / 3.0 * VDC }, { 0.0, 0.0 } };

const unsigned int swt[8][3] = { { 0, 0, 0 }, { 0, 0, 1 }, { 0, 1, 0 }, { 0, 1,
		1 }, { 1, 0, 0 }, { 1, 0, 1 }, { 1, 1, 0 }, { 1, 1, 1 } };

void inv_init(Inverter *inv) {
	inv->f_out = 0.0;
	inv->f_pwm = 10e3;
	inv->f_sampling = 1500;
	inv->Vm = 0.0;
	inv->theta = 0.0;
	inv->dc[U] = 0.0;
	inv->dc[V] = 0.0;
	inv->dc[W] = 0.0;
}

void inv_calc_dc(Inverter *inv) {
	double Vref[2] = { inv->Vm * cos(inv->theta), inv->Vm * sin(inv->theta) };
	double Vl[2] = { 0.0 };
	double Vr[2] = { 0.0 };
	unsigned int swl[3] = { 0 };
	unsigned int swr[3] = { 0 };

	double T_sampling = 1.0 / inv->f_sampling;

	if (inv->theta >= 0.0 && inv->theta <= PI / 3.0) {
		Vl[0] = Vi[6][0];
		Vl[1] = Vi[6][1];

		Vr[0] = Vi[4][0];
		Vr[1] = Vi[4][1];

		swl[0] = swt[6][0];
		swl[1] = swt[6][1];
		swl[2] = swt[6][2];

		swr[0] = swt[4][0];
		swr[1] = swt[4][1];
		swr[2] = swt[4][2];
	} else if (inv->theta >= PI / 3 && inv->theta <= 2.0 / 3.0 * PI) {
		Vl[0] = Vi[2][0];
		Vl[1] = Vi[2][1];

		Vr[0] = Vi[6][0];
		Vr[1] = Vi[6][1];

		swl[0] = swt[2][0];
		swl[1] = swt[2][1];
		swl[2] = swt[2][2];

		swr[0] = swt[6][0];
		swr[1] = swt[6][1];
		swr[2] = swt[6][2];
	} else if (inv->theta >= 2.0 / 3.0 * PI && inv->theta <= PI) {
		Vl[0] = Vi[3][0];
		Vl[1] = Vi[3][1];

		Vr[0] = Vi[2][0];
		Vr[1] = Vi[2][1];

		swl[0] = swt[3][0];
		swl[1] = swt[3][1];
		swl[2] = swt[3][2];

		swr[0] = swt[2][0];
		swr[1] = swt[2][1];
		swr[2] = swt[2][2];
	} else if (inv->theta >= PI && inv->theta <= 4.0 / 3.0 * PI) {
		Vl[0] = Vi[1][0];
		Vl[1] = Vi[1][1];

		Vr[0] = Vi[3][0];
		Vr[1] = Vi[3][1];

		swl[0] = swt[1][0];
		swl[1] = swt[1][1];
		swl[2] = swt[1][2];

		swr[0] = swt[3][0];
		swr[1] = swt[3][1];
		swr[2] = swt[3][2];
	} else if (inv->theta >= 4.0 / 3.0 * PI && inv->theta <= 5.0 / 3.0 * PI) {
		Vl[0] = Vi[5][0];
		Vl[1] = Vi[5][1];

		Vr[0] = Vi[1][0];
		Vr[1] = Vi[1][1];

		swl[0] = swt[5][0];
		swl[1] = swt[5][1];
		swl[2] = swt[5][2];

		swr[0] = swt[1][0];
		swr[1] = swt[1][1];
		swr[2] = swt[1][2];
	} else if (inv->theta >= 5.0 / 3.0 * PI && inv->theta <= 2.0 * PI) {
		Vl[0] = Vi[4][0];
		Vl[1] = Vi[4][1];

		Vr[0] = Vi[5][0];
		Vr[1] = Vi[5][1];

		swl[0] = swt[4][0];
		swl[1] = swt[4][1];
		swl[2] = swt[4][2];

		swr[0] = swt[5][0];
		swr[1] = swt[5][1];
		swr[2] = swt[5][2];
	}

	double A[2][2] = { { Vl[0], Vr[0] }, { Vl[1], Vr[1] } };

	double detA = A[0][0] * A[1][1] - A[0][1] * A[1][0];
	double invA[2][2] = { { A[1][1] / detA, -A[0][1] / detA }, { -A[1][0]
			/ detA, A[0][0] / detA } };
	double b[2] = { T_sampling * Vref[0], T_sampling * Vref[1], };

	double T[2] = { invA[0][0] * b[0] + invA[0][1] * b[1], invA[1][0] * b[0]
			+ invA[1][1] * b[1], };

	double Toff = T_sampling - T[0] - T[1];

	inv->dc[U] = 1.0 / T_sampling
			* (swl[0] * T[0] + swr[0] * T[1] + Toff / 2.0);
	inv->dc[V] = 1.0 / T_sampling
			* (swl[1] * T[0] + swr[1] * T[1] + Toff / 2.0);
	inv->dc[W] = 1.0 / T_sampling
			* (swl[2] * T[0] + swr[2] * T[1] + Toff / 2.0);
}
