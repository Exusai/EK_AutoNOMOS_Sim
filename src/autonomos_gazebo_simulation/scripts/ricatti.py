import sys
import numpy as np
import scipy.linalg
# Frec. de muestreo de la senal de control(Hz)
f = 30.0   
# Periodo de muestreo de la senal de control (s)
h = 1/f
# Velocidad de avance del auto (rpm) 
vrpm = 800
# Velocidad de avance del auto (m/s)
v = vrpm*(0.588/200)
# Separacion de los ejes (m)
L = 0.26
# Distancia entre las llantas traseras y la base de la homografia (m)
Lh = 0.28
# Matrices del modelo cinematico usado
A = np.matrix ([[1, v*h, 0], [0, 1, 0],[h, (v*h**2)/2, 1]])
B = (v*h/L)*np.matrix([[Lh+(v*h)/2], [1], [(v*h**2)/6+(h*Lh)/2]])
# Matrices de ponderacion propuestas
Q = 0.02*np.matrix([[1, 0, 0],[0, 1, 0],[0, 0, 1]]) # dir. prop. a K
R = 24.0	# inv. prop. a K
print("-------------------------------------------")
print('v_rpm',vrpm)
print('v',v)
print('R',R)
print('Q',Q)
print("-------------------------------------------")
P = np.matrix(scipy.linalg.solve_discrete_are(A,B,Q,R))
#print('P',P)
print("-------------------------------------------")
print('Ganancias del Optimal Controller Ke Kth')
K = np.matrix(scipy.linalg.inv(B.T*P*B+R)*(B.T*P*A))
print('K',K)
