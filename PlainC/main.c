#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <assert.h>

#define PSEC 1.0e12
#define NSEC 1.0e9
#define USEC 1.0e6
#define MSEC 1.0e3
#define  SEC 1.0
#define SC USEC

double randbetween(double lower, double upper) {
    double sc = rand() / (double) RAND_MAX;
    return lower + sc * (upper - lower);
}

double ftri(double x, double p, double a)
{
    return 4 * a / p * fabs(fmod(x -p / 4, p)  - p/ 2) -a;
}

int main() {
    FILE *fp = fopen("export.csv", "w");
    srand(time(NULL));

    double Vdc = 50.0;
    double Vm = 20.0;

    double f_pwm = 10.0e3;
    double f_out = 50.0;
    double f_update = 1000;
    double f_simulation = 500e3;
    assert(f_simulation >= 2.0 * f_pwm);


    double T_pwm = 1.0 / f_pwm;
    double T_out = 1.0 / f_out;
    double T_update = 1.0 / f_update;
    double T_simulation = 1.0 / f_simulation;
    double dt = T_simulation;

    printf("T_pwm: %f s\n", T_pwm);
    printf("T_out: %f s\n", T_out);
    printf("T_update: %f s\n", T_update);
    printf("T_simulation: %f s\n", T_simulation);
    printf("dt: %f s\n", dt);

    double t0 = 0.0, t1 = 0.0;
    double dc[3] = {0.0};

    do {
        //t1 += randbetween(T_simulation / 2.0, T_simulation);
        t1 += dt;
        double elapsed = t1 - t0;

        if (elapsed >= T_pwm || t0< 1.0e-10) {
            t0 = t1;

            double V[3] = {
                    Vm * sin(2 * M_PI * f_out * t1 + 0 * M_PI * 2.0 / 3.0),
                    Vm * sin(2 * M_PI * f_out * t1 + 1 * M_PI * 2.0 / 3.0),
                    Vm * sin(2 * M_PI * f_out * t1 + 2 * M_PI * 2.0 / 3.0)
            };

            double Vref[2] = {
                    1.0 / 3.0 * (2.0 * V[0] - V[1] - V[2]),
                    M_SQRT3 / 3.0 * (V[1] - V[0])
            };

            double theta = atan2(Vref[1], Vref[0]) * 180.0 / M_PI;

            unsigned char swl[3];
            unsigned char swr[3];

            if (theta >= 0.0 && theta <= 60.0) {
                swl[0] = 1;
                swl[1] = 1;
                swl[2] = 0;
                swr[0] = 1;
                swr[1] = 0;
                swr[2] = 0;
            } else if (theta >= 60.0 && theta <= 120.0) {
                swl[0] = 0;
                swl[1] = 1;
                swl[2] = 0;
                swr[0] = 1;
                swr[1] = 1;
                swr[2] = 0;
            } else if (theta >= 120.0 && theta <= 180.0) {
                swl[0] = 0;
                swl[1] = 1;
                swl[2] = 1;
                swr[0] = 0;
                swr[1] = 1;
                swr[2] = 0;
            } else if (theta <= -120.0 && theta >= -180.0) {
                swl[0] = 0;
                swl[1] = 0;
                swl[2] = 1;
                swr[0] = 0;
                swr[1] = 1;
                swr[2] = 1;
            } else if (theta <= -60.0 && theta >= -120.0) {
                swl[0] = 1;
                swl[1] = 0;
                swl[2] = 1;
                swr[0] = 0;
                swr[1] = 0;
                swr[2] = 1;
            } else if (theta <= 0.0 && theta >= -60.0) {
                swl[0] = 1;
                swl[1] = 0;
                swl[2] = 0;
                swr[0] = 1;
                swr[1] = 0;
                swr[2] = 1;
            }
            double Vl[2] = {
                    Vdc / 3.0 * (2 * swl[0] - swl[1] - swl[2]),
                    Vdc * M_SQRT3 / 3.0 * (swl[1] - swl[2])
            };
            double Vr[2] = {
                    Vdc / 3.0 * (2 * swr[0] - swr[1] - swr[2]),
                    Vdc * M_SQRT3 / 3.0 * (swr[1] - swr[2])
            };

            double A[2][2] = {
                    {Vl[0], Vr[0]},
                    {Vl[1], Vr[1]}
            };
            double detA = A[0][0] * A[1][1] - A[0][1] * A[1][0];

            double invA[2][2] = {
                    {A[1][1]/detA, -A[0][1]/detA},
                    {-A[1][0]/detA, A[0][0]/detA}
            };

            double b[2] = {
                    T_pwm * Vref[0],
                    T_pwm * Vref[1]
            };

            double T[2] = {
                invA[0][0] * b[0] + invA[0][1] * b[1],
                invA[1][0] * b[0] + invA[1][1] * b[1]
            };

            double Toff = T_pwm - T[0] - T[1];

            dc[0] = (swl[0]*T[0] + swr[0]*T[1] + Toff / 2) / T_pwm;
            dc[1] = (swl[1]*T[0] + swr[1]*T[1] + Toff / 2) / T_pwm;
            dc[2] = (swl[2]*T[0] + swr[2]*T[1] + Toff / 2) / T_pwm;
        }

        double vtr = (0.5 + 0.5 * ftri(t1+T_pwm / 4, T_pwm, 1.0));
        double pwm[3] = {0.0};
        if (dc[0] >= vtr) { pwm[0] = 1.0;}
        if (dc[1] >= vtr) { pwm[1] = 1.0;}
        if (dc[2] >= vtr) { pwm[2] = 1.0;}



        //printf("t1: %.15f\n", t1);
        fprintf(fp, "%.15f, %f,%f,%f,", t1, dc[0], dc[1], dc[2]);
        fprintf(fp, "%f,%f,%f, %f\n", pwm[0], pwm[1], pwm[2], vtr);

    } while (t1 <= 2 * T_out);

    fclose(fp);


    return 0;
}
