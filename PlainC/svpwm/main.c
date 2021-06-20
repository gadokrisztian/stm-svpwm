#include <stdio.h>
#include "inverter.h"


int main() {
    Inverter inv;
    inv_init(&inv);
    int n = 10;
    for(int i = 0; i < n; i++)
    {
        double theta = i * 2 * M_PI / n;
        double vm = 20;
        inv_calc_dc(vm, theta, &inv);
        printf("%3.3f \tU: %.4f \tV: %.4f \tW: %.4f\n", 180.0 / M_PI*theta, inv.dc[U], inv.dc[V], inv.dc[W]);
    }

  return 0;
}
