import numpy as np
from math import pi
from roboticstoolbox.models.DH import UR3
import roboticstoolbox as rtb
from spatialmath import SE3

left = rtb.models.DH.Baxter('left')   # Baxter left arm
right = rtb.models.DH.Baxter('right') # Baxter right arm
left.base = SE3(0.064614, 0.25858, 0.119) * SE3.Rx(pi / 4)
right.base = SE3(0.063534, -0.25966, 0.119) * SE3.Rx(-pi / 4)

qLeft = [0,0,0,0,0,0,0]
qRight = [0,0,0,0,0,0,0]

T_left = left.fkine(qLeft)
T_right = right.fkine(qRight)

a = T_left.t 
b = T_right.t

distance = np.linalg.norm(a - b)

print(distance*1000, "mm")