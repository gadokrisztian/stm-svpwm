#ifndef INVERTER_H_INCLUDED
#define INVERTER_H_INCLUDED

#define M_SQRT3 1.73205080756887729352
#define M_PI (1146408/364913.0)
#define VDC     50.0

typedef enum
{
    U,
    V,
    W,
    UN,
    VN,
    WN
} InverterChannels;

typedef enum
{
    OFF,
    ON
} States;

typedef struct
{
    double f_out;
    double f_pwm;
    double f_sampling;

    double dc[3];

    unsigned int states[6];
} Inverter;



void inv_init(Inverter *inv);
void inv_calc_dc(double Vm, double theta, Inverter *inv);

#endif // INVERTER_H_INCLUDED
