#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

#define PSEC 1.0e12
#define NSEC 1.0e9
#define USEC 1.0e6
#define MSEC 1.0e3
#define  SEC 1.0
#define SC USEC

double randbetween(double lower, double upper)
{
    double sc = rand() / (double ) RAND_MAX;
    return lower + sc * (upper - lower);
}

int main() {
    FILE *fp = fopen("export.csv", "w");
    srand(time(NULL));



    double f_out = 50;
    double f_sampling = 1000;
    double f_update = 2.5 * f_out;
    double omega = 2 * M_PI * f_out;

    double Vdc = 50.0;
    double Vm = 22.0;


    double T_out = SC / f_out;
    double T_sampling = SC / f_sampling;
    double T_update = SC / f_update;
    printf("%.3lf Hz -> T =  %.3lf\n", f_out, T_out);

    unsigned long updatecounter = 0;
    printf("sampling period: %.4lf us\n", T_sampling);

    double t0 = 0.0, t1 = 0.0;

    double dc[3] = {0, 0, 0};

    do {
        t1 += randbetween(0.0, 1e-3);
        double elapsed = t1 - t0;

        if (elapsed >= f_update / SC) {

            double V[3] = {
                    Vm * sin(omega * t1 + 0 * M_2_PI / 3.0),
                    Vm * sin(omega * t1 + 1 * M_2_PI / 3.0),
                    Vm * sin(omega * t1 + 2 * M_2_PI / 3.0),
            };

            double Vref[2] = {
                    1.0/3.0 * (2.0*V[0] - V[1] - V[2]),
                    M_SQRT3/3.0 * (V[1]-V[2])
            };

            double theta = atan2(Vref[1], Vref[0]) * 180.0 / M_PI;

            unsigned char swl[3];
            unsigned char swr[3];

            if (theta >= 0.0 && theta <= 60.0) {
                swl[0] = 1; swl[1] = 1, swl[2] = 0;
                swr[0] = 1; swr[1] = 0, swr[2] = 0;
            } else if (theta >= 60.0 && theta <= 120.0) {
                swl[0] = 0; swl[1] = 1, swl[2] = 0;
                swr[0] = 1; swr[1] = 1, swr[2] = 0;
            } else if (theta >= 120.0 && theta <= 180.0) {
                swl[0] = 0; swl[1] = 1, swl[2] = 1;
                swr[0] = 0; swr[1] = 1, swr[2] = 0;
            } else if (theta <= -120.0 && theta >= -180.0) {
                swl[0] = 0; swl[1] = 0, swl[2] = 1;
                swr[0] = 0; swr[1] = 1, swr[2] = 1;
            } else if (theta <= -60.0 && theta >= -120.0) {
                swl[0] = 1; swl[1] = 0, swl[2] = 1;
                swr[0] = 0; swr[1] = 0, swr[2] = 1;
            } else if (theta <= -0.0 && theta >= -60.0) {
                swl[0] = 1; swl[1] = 0, swl[2] = 0;
                swr[0] = 1; swr[1] = 0, swr[2] = 1;
            }

            double Vl[2] = {
                    Vdc / 3.0 *(2*swl[0] - swl[1] - swl[2]),
                    Vdc * M_SQRT3 / 3.0 * (swl[1] - swl[2])
            };
            double Vr[2] = {
                    Vdc / 3.0 *(2*swr[0] - swr[1] - swr[2]),
                    Vdc * M_SQRT3 / 3.0 * (swr[1] - swr[2])
            };

            double A[4] = {Vl[0], Vr[0],
                           Vl[1], Vr[1]};
            double detA = A[0] * A[3] - A[2] * A[1];
            double invA[4] = {A[3] / detA, -A[1] / detA, -A[2] / detA, A[0] / detA};

            double T[2] = {
                    T_sampling * (Vref[0] * invA[0] + Vref[1] * invA[1]),
                    T_sampling * (Vref[0] * invA[2] + Vref[1] * invA[3])
            };

            double Toff = T_sampling - T[0] - T[1];
            dc[0] = (swl[0]*T[0] + swr[0]*T[1] + Toff / 2) / T_sampling;
            dc[1] = (swl[1]*T[0] + swr[1]*T[1] + Toff / 2) / T_sampling;
            dc[2] = (swl[2]*T[0] + swr[2]*T[1] + Toff / 2) / T_sampling;



            printf("%.20lf us -> %.5lf\n", elapsed, 1.0e6 / elapsed);
            updatecounter++;
            t0 = t1;
            //printf("%d\n", updatecounter);
        }
        fprintf(fp, "%.20lf,%f,%f,%f\n", t1 / SC, dc[0], dc[1],dc[2]);
    } while (t1 / SC <= 0.05);

    fclose(fp);


    return 0;
}
