
# dataclass
from dataclasses import dataclass
from math import sqrt, fabs

def fsgn(x):
    if x > 0:
        return 1
    elif x < 0:
        return -1
    else:
        return 0
    
def fsg(x, d):
    return ((fsgn(x + d) - fsgn(x - d)) / 2)

def fst(x1_delta, x2, r, h0):
    d = r * h0 * h0
    a0 = h0 * x2
    y = x1_delta + a0
    a1 = sqrt(d * (d + 8 * fabs(y)))
    a2 = a0 + fsgn(y) * (a1 - d) / 2
    a = (a0 + y) * fsg(y, d) + a2 * (1 - fsg(y, d))

@dataclass
class TdFunction:
    r : float
    h : float
    h0 : float

    x: float
    y: float
    z: float

    def update(self, ref):
        last_v1 = self.v1
        last_v2 = self.v2
        self.v1 = last_v1 + self.h * last_v2
        self.v2 = last_v2 + self.h * fst(last_v1 - self.ref, last_v2, self.r, self.h0)