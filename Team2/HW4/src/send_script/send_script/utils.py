import numpy as np

def Rz(psi):
    c, s = np.cos, np.sin
    return np.array([
        [c(psi), -s(psi), 0], 
        [s(psi),  c(psi), 0], 
        [0, 0, 1]])