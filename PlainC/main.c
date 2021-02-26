#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>


double randbetween(double lower, double upper) {
    double sc = rand() / (double) RAND_MAX;
    return lower + sc * (upper - lower);
}

double ftri(double x, double p, double a) {
    return 4 * a / p * fabs(fmod(x - p / 4, p) - p / 2) - a;
}

int main() {
    FILE *fp = fopen("export.csv", "w");
    srand(time(NULL));

    double Vdc = 50.0;
    double Vm = 28.0;
    double dtheta = 0.001;
    double theta = 0.0;

    double T_update = 1.0 / 500.0;

    double Vi[8][2] = {
            {0.0,              0.0},
            {-1.0 / 3.0 * Vdc, -M_SQRT3 / 3.0 * Vdc},
            {-1.0 / 3.0 * Vdc, M_SQRT3 / 3.0 * Vdc},
            {-2.0 / 3.0 * Vdc, 0.0},
            {2.0 / 3.0 * Vdc,  0.0},
            {1.0 / 3.0 * Vdc,  -M_SQRT3 / 3.0 * Vdc},
            {1.0 / 3.0 * Vdc,  M_SQRT3 / 3.0 * Vdc},
            {0.0,              0.0},
    };

    unsigned int swt[8][3] = {
            {0, 0, 0},
            {0, 0, 1},
            {0, 1, 0},
            {0, 1, 1},
            {1, 0, 0},
            {1, 0, 1},
            {1, 1, 0},
            {1, 1, 1}
    };

    do {
        printf("%f\n", theta);

        double Vref[2] = {
                Vm * cos(theta),
                Vm * sin(theta)
        };
        double Vl[2] = {0.0};
        double Vr[2] = {0.0};
        unsigned int swl[3] = {0};
        unsigned int swr[3] = {0};

        if (theta >= 0.0 && theta <= M_PI / 3.0) {
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
        } else if (theta >= M_PI / 3 && theta <= 2.0 / 3.0 * M_PI) {
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
        } else if (theta >= 2.0 / 3.0 * M_PI && theta <= M_PI) {
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
        } else if (theta >= M_PI && theta <= 4.0 / 3.0 * M_PI) {
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
        } else if (theta >= 4.0 / 3.0 * M_PI && theta <= 5.0 / 3.0 * M_PI) {
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
        } else if (theta >= 5.0 / 3.0 * M_PI && theta <= 2.0 * M_PI) {
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

        double A[2][2] = {
                {Vl[0], Vr[0]},
                {Vl[1], Vr[1]}
        };

        double detA = A[0][0] * A[1][1] - A[0][1] * A[1][0];
        double invA[2][2] = {
                {A[1][1] / detA,  -A[0][1] / detA},
                {-A[1][0] / detA, A[0][0] / detA}
        };
        double b[2] = {
                T_update * Vref[0],
                T_update * Vref[1],
        };

        double T[2] = {
                invA[0][0] * b[0] + invA[0][1] * b[1],
                invA[1][0] * b[0] + invA[1][1] * b[1],
        };

        double Toff = T_update - T[0] - T[1];
        double dc[3] = {
                1.0 / T_update * (swl[0] * T[0] + swr[0] * T[1] + Toff / 2.0),
                1.0 / T_update * (swl[1] * T[0] + swr[1] * T[1] + Toff / 2.0),
                1.0 / T_update * (swl[2] * T[0] + swr[2] * T[1] + Toff / 2.0),
        };

        fprintf(fp, "%f,%f,%f,%f\n", theta, dc[0], dc[1], dc[2]);

        theta += dtheta;
    } while (theta <= 2 * M_PI);


    fclose(fp);


    return 0;
}
