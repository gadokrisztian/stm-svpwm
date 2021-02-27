from math import sqrt, pi, atan2


class SVPWM:
    Vi= [
        [0, 0],
        [-1.0/3.0, -sqrt(3) / 3.0],
        [-1.0/3.0, sqrt(3) / 3.0],
        [-2.0 / 3.0, 0.0],
        [2.0 / 3.0, 0.0],
        [1.0 / 3.0, -sqrt(3) / 3.0],
        [1.0 / 3.0, sqrt(3) / 3.0],
        [0, 0]
        ]
    swt = [[0, 0, 0],
           [0, 0, 1],
           [0, 1, 0],
           [0, 1, 1],
           [1, 0, 0],
           [1, 0, 1],
           [1, 1, 0],
           [1, 1, 1]]
    def __init__(self, vdc, tupdate):
        self.Vdc = vdc
        self.T_update = tupdate

    def getDC(self, Vref):
        assert(sqrt(Vref[0]**2 + Vref[1]**2) <= self.Vdc * sqrt(3)/3)

        theta = atan2(Vref[1], Vref[0]) * 180.0 / pi
        Vl = []
        Vr = []
        swl = []
        swr = []

        if theta >=0 and theta <=60:
            s = 1
            Vl = SVPWM.Vi[6].copy()
            Vr = SVPWM.Vi[4].copy()
            swl =SVPWM.swt[6].copy()
            swr = SVPWM.swt[4].copy()
        elif theta >=60 and theta <=120:
            s = 2
            Vl = SVPWM.Vi[2].copy()
            Vr = SVPWM.Vi[6].copy()
            swl = SVPWM.swt[2].copy()
            swr = SVPWM.swt[6].copy()
        elif theta >=120 and theta <=180:
            s = 3
            Vl = SVPWM.Vi[3].copy()
            Vr = SVPWM.Vi[2].copy()
            swl = SVPWM.swt[3].copy()
            swr = SVPWM.swt[2].copy()
        elif theta >=-180 and theta <=-120:
            s = 4
            Vl = SVPWM.Vi[1].copy()
            Vr = SVPWM.Vi[3].copy()
            swl = SVPWM.swt[1].copy()
            swr = SVPWM.swt[3].copy()
        elif theta >=-120 and theta <=-60:
            s = 5
            Vl = SVPWM.Vi[5].copy()
            Vr = SVPWM.Vi[1].copy()
            swl = SVPWM.swt[5].copy()
            swr = SVPWM.swt[1].copy()
        else:
            s=6
            Vl = SVPWM.Vi[4].copy()
            Vr = SVPWM.Vi[5].copy()
            swl = SVPWM.swt[4].copy()
            swr = SVPWM.swt[5].copy()

        A = [
            [Vl[0]*self.Vdc, Vr[0]*self.Vdc],
            [Vl[1]*self.Vdc, Vr[1]*self.Vdc]
            ]

        detA = A[0][0]*A[1][1] - A[1][0]*A[0][1]
        invA = [
            [A[1][1] / detA, -A[0][1] / detA],
            [-A[1][0] / detA, A[0][0] / detA]
        ]
        b = [self.T_update * Vref[0], self.T_update * Vref[1]]

        T = [
            invA[0][0] * b[0] + invA[0][1] * b[1],
            invA[1][0] * b[0] + invA[1][1] * b[1]
        ]
        Toff = self.T_update - T[0] - T[1]

        dc = [
            1/self.T_update *(swl[0] * T[0] + swr[0] * T[1] + Toff / 2.0),
            1/self.T_update *(swl[1] * T[0] + swr[1] * T[1] + Toff / 2.0),
            1/self.T_update *(swl[2] * T[0] + swr[2] * T[1] + Toff / 2.0),
        ]

        return dc
