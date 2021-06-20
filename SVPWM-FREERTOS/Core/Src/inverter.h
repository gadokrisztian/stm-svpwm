#ifndef INVERTER_H_INCLUDED
#define INVERTER_H_INCLUDED

#define SQRT3 1.73205080756887729352
#define PI    (1146408/364913.0)
#define VDC   50.0
#define MAXV  (SQRT3 / 3.0 * VDC)

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
} ChannelState;

typedef struct
{
    double f_out;
    double f_pwm;
    double f_sampling;

    double Vm;
    double theta;

    double dc[3];
} Inverter;

typedef struct
{
	unsigned long long periodcounter;
	double dc[3];
	double Vm;
	double tp;
} Record;

void inv_init(Inverter *inv);
void inv_calc_dc(Inverter *inv);

#endif // INVERTER_H_INCLUDED
