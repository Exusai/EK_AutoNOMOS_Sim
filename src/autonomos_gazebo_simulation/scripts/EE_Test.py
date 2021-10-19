import numpy as np
import cv2
from keras.models import load_model
from model_builder import get_KBW
from lib_EE_driver_01 import ee_driver

# Importación de la red entrenada
print("*******************************************************")
print("*******************************************************")
print("*******************************************************")
h5_path = '/home/sherlock/CNN_venv/scripts/tensor_test/EE_learning_p3.5/EE_learning_model.h5'
model = load_model(h5_path)
B_c, W_c, B_d, W_d = get_KBW(h5_path)
print("*******************************************************")
print("*******************************************************")
print("*******************************************************")

imagen0 = cv2.imread('/home/sherlock/CNN_venv/scripts/tensor_test/EE_learning_p3.5/EE_dataset/300_img.jpg')
cv2.imshow("Original",imagen0)
Nx, Ny,ch = imagen0.shape
imagen1 = imagen0
for i in range(Nx):
	for j in range(Ny):
		if (imagen0[i,j,0]<=20) and (imagen0[i,j,2]<=20) and (imagen0[i,j,1]>=0): imagen1[i,j,:]=(0,0,0)
_,imagen1 = cv2.threshold(imagen1,127,255,cv2.THRESH_BINARY)
#imagen1 = cv2.cvtColor(imagen0, cv2.COLOR_BGR2YUV)
imagen1 = imagen1[279:480,0:640]
nx = 100
ny = 320
imagen1 = cv2.resize(imagen1,(ny,nx))
cv2.imshow("Sin verde",imagen1)
Ymax = 40.0
imagen1 = (1.0/127.5)*np.array(imagen1)
imagen1 = np.add(-1.0,imagen1)

# Metodo 1 TF
imagen_TF = imagen1.reshape(1,nx,ny,3)
u_norm1 = model.predict(imagen_TF)
u1 = (u_norm1)*(Ymax)+90.0
print("u1 ",u1)

# Metodo 2 MyL
u_norm2 = ee_driver(B_c,W_c,B_d,W_d,imagen1)
u2 = (u_norm2)*(Ymax)+90.0
print("u2 ",u2)

cv2.waitKey(0)
cv2.destroyAllWindows()



