import numpy as np
import matplotlib.pyplot as plt
import cmath as cm
from numpy import pi

def ftri(x, p, a):
    return 4 * a / p * np.abs((x -p / 4) % p  - p/ 2) -a

f = 50
fs = 7e3
Vdc = 50
Vm = 20


UU = lambda x: Vm * np.sin(2 * np.pi * f * x + 0 * 2 * np.pi / 3)
UV = lambda x: Vm * np.sin(2 * np.pi * f * x + 1 * 2 * np.pi / 3)
UW = lambda x: Vm * np.sin(2 * np.pi * f * x + 2 * 2 * np.pi / 3)

# -----------------------------------------------------------------------------------------------

plt.style.use(['default', 'fivethirtyeight'])
plt.rcParams['lines.linewidth'] = 2
plt.rcParams['font.size'] = 10

systick = np.linspace(0, 40, 35001)*1e-3
dt = systick[1] - systick[0]
w = 2 * np.pi * f
Vrefmax = Vdc * np.sqrt(3) / 3
T = 1 / f
Ts = 1 / fs
print(f'Vdc: {Vdc:.4f} V')
print(f'Vref max: {Vrefmax:.4f} V')
print(f'sampling period: {Ts*1e6:.4f} us')
print(f'dt: {dt*1e6:.4f} us')
#if Vrefmax <= Vm: raise ValueError("Vm nagyobb, mint Vrefmax")
switchtable = np.array([[0,1,0,1,0,1,0,1], [0,0,1,1,0,0,1,1], [0,0,0,0,1,1,1,1]]).T
MClark = 2 / 3 * np.array([
    [1, -0.5, -0.5],
    [0, np.sqrt(3) / 2, -np.sqrt(3) / 2],
])
Ml2n = Vdc / 3 * np.array([
    [2, -1, -1],
    [-1, 2, -1],
    [-1, -1, 2]
])

Ml2l = Vdc * np.array([
    [1, -1, 0],
    [0, 1, -1],
    [-1, 0, 1]
])

MV = Ml2n.copy()

U = np.ones_like(systick)
V = np.ones_like(systick)
W = np.ones_like(systick)

U = np.zeros_like(systick)
V = np.zeros_like(systick)
W = np.zeros_like(systick)

Tsi = np.zeros((systick.shape[0], 3))
vtrii = np.zeros_like(systick)
s = 1


dPeriod = int(np.ceil(Ts/dt))
cmpr=1
for i, ti in enumerate(systick):
    if i % dPeriod == 0:
        Vref = np.dot(MClark, np.array([UU(ti), UV(ti), UW(ti)]))
        theta = np.arctan2(Vref[1], Vref[0]) * 180 / np.pi
        if theta >= 0  and theta <= 60:
            s = 1
            swl = np.array([1, 1, 0])
            swr = np.array([1, 0, 0])
            Vl = np.dot(MClark, np.dot(MV, swl))
            Vr = np.dot(MClark, np.dot(MV, swr))
            
        elif theta >= 60  and theta <= 120:
            s = 2
            swl = np.array([0, 1, 0])
            swr = np.array([1, 1, 0])
            Vl = np.dot(MClark, np.dot(MV, swl))
            Vr = np.dot(MClark, np.dot(MV, swr))
            
        elif theta >= 120  and theta <= 180:
            s = 3
            swl = np.array([0, 1, 1])
            swr = np.array([0, 1, 0])
            Vl = np.dot(MClark, np.dot(MV, swl))
            Vr = np.dot(MClark, np.dot(MV, swr))
            
        elif theta <= -120  and theta >= -180:
            s = 4
            swl = np.array([0, 0, 1])
            swr = np.array([0, 1, 1])
            Vl = np.dot(MClark, np.dot(MV, swl))
            Vr = np.dot(MClark, np.dot(MV, swr))
            
        elif theta <= -60  and theta >= -120:
            s = 5
            swl = np.array([1, 0, 1])
            swr = np.array([0, 0, 1])
            Vl = np.dot(MClark, np.dot(MV, swl))
            Vr = np.dot(MClark, np.dot(MV, swr))
            
        elif theta <= 0  and theta >= -60:
            s = 6
            swl = np.array([1, 0, 0])
            swr = np.array([1, 0, 1])
            Vl = np.dot(MClark, np.dot(MV, swl))
            Vr = np.dot(MClark, np.dot(MV, swr))
    
        A = np.array([Vl, Vr]).T
        T = np.linalg.solve(A, Ts * Vref)
        Toff = Ts - T.sum()
        cmpr = (np.dot(np.array([swl, swr]).T, T) +  Toff / 2).ravel()
    

    vtri = Ts*(0.5 + 0.5 * ftri(ti+Ts/4, Ts, 1))
    Tsi[i] = cmpr
    vtrii[i] = vtri
    
    if cmpr[0] >= vtri:
        U[i] = 1.0
        
    if cmpr[1] >= vtri:
        V[i] = 1.0
        
    if cmpr[2] >= vtri:
        W[i] = 1.0
