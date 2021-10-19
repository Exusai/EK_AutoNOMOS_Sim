import os
import re
import cv2
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from keras.models import Sequential
from keras.layers import Dense, Dropout, Flatten, Lambda
from keras.layers.convolutional import Conv2D, MaxPooling2D
from keras.optimizers import Adagrad
from keras.optimizers import Adam
from keras.optimizers import SGD	
from sklearn.model_selection import train_test_split

from keras.utils import Sequence
#*********************************************************************************************
#*********************************************************************************************
#*********************************************************************************************
################# Parametros #####################
# No debe haber otro archivo mas que imagenes en las carpetas del dataset
path = "/home/sherlock/CNN_venv/scripts/tensor_test/EE_learning_p3.5/EE_dataset" # Folder con el dataset
outFile = 'steering.csv'
batch_size_val = 4  # Cuantos se procesan juntos/Cuantos se usan para actualizar los pesos
epochs_val= 50 # Numero de epocas/Criterio de paro
validationRatio = 0.2 # Del 80% restante, usa otro 20% de los datos como conjunto de validacion
nx = 0
ny = 0
Ymax = 40.0 #42.23

#*********************************************************************************************
#*********************************************************************************************
#*********************************************************************************************
################# CREA LA RED NEURONAL CONVOLUCIONAL #################
def build_CNN():
	# NVIDIA_CNN_model
	model = Sequential()
	model.add((Conv2D(12,(5,5),input_shape=(nx,ny,ch),activation='relu'))) #24
	model.add(MaxPooling2D(pool_size=(2,2)))
	model.add(Conv2D(24,(5,5), activation='relu')) #36
	model.add(MaxPooling2D(pool_size=(2,2)))
	model.add(Conv2D(36,(3,3), activation='relu')) #48
	model.add(MaxPooling2D(pool_size=(5,5)))
	#model.add(Conv2D(64,(3,3), activation='relu'))
	#model.add(Conv2D(64,(3,3), activation='relu'))
	#model.add(Dropout(0.5))
	model.add(Flatten())
	#	model.add(Dense(100, activation='relu'))
	model.add(Dense(25, activation='relu'))
	model.add(Dense(10, activation='relu'))
	model.add(Dense(1))
	model.compile(loss='mean_squared_error', optimizer=Adam(lr=0.001),metrics=['mean_absolute_error'])
	#model.compile(loss='mean_squared_error', optimizer=SGD(lr=0.01,momentum=0.001),metrics=['mean_absolute_error'])
	model.summary()
	return model
#*********************************************************************************************
#*********************************************************************************************
#*********************************************************************************************
class Data_Generator(Sequence) : #keras.utils.Sequence
	def __init__(self, x_list, y_list, batch_size):
		self.x_list = x_list
		self.y_list = y_list
		self.batch_size = batch_size
	def __len__(self) :
		return (np.ceil(len(self.x_list) / float(self.batch_size))).astype(np.int)
	def __getitem__(self, idx) :
		batch_x = self.x_list[idx * self.batch_size : (idx+1) * self.batch_size]
		batch_y = self.y_list[idx * self.batch_size : (idx+1) * self.batch_size]
		train_image = []
		for k in range(0, len(batch_x)):
			img = cv2.imread(path+'/'+batch_x[k])
			img = img[279:480,0:640]
			img = cv2.resize(img,(ny,nx))
			for i in range(nx): 			# Elimina la linea verde
				for j in range(ny):
					if (img[i,j,0]<=20) and (img[i,j,2]<=20) and (img[i,j,1]>=0): img[i,j,:]=(0,0,0)
			#img = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)
			_,img = cv2.threshold(img,127,255,cv2.THRESH_BINARY)
			train_image.append(img)
		# Reacomodo en forma tensorial y normalizacion
		imagesX = (1.0/127.5)*np.array(train_image)
		imagesX = np.add(-1.0,imagesX)
		steeringY = np.add(-90.0,np.array(batch_y))
		steeringY = (1.0/Ymax)*steeringY
		return imagesX, steeringY

#*********************************************************************************************
#*********************************************************************************************
#*********************************************************************************************
################# PRINCIPAL #################
# Guarda el nombre de las imagenes en una lista y las ordena
X_list = os.listdir(path)
X_list.sort(key=lambda var:[int(x) if x.isdigit() else x for x in re.findall(r'[^0-9]|[0-9]+', var)])
# Lee el archivo CSV
Y_list=pd.read_csv(outFile)
steering_list = Y_list['steering'].values
# Tamano del lote
batch_size = len(X_list)
# Genera los datos de entrenamiento y de prueba
X_train, X_val, steering_train, steering_val = train_test_split(X_list,steering_list, test_size=validationRatio)

# Imagen de ejemplo para definir algunos parametros
img_ex = cv2.imread(path+'/'+X_train[50])
img_ex = img_ex[279:480,0:640]
nx,ny,ch = img_ex.shape
nx = int(nx/2.0)
ny = int(ny/2.0)
img_ex = cv2.resize(img_ex,(ny,nx))
for i in range(nx): # Quita la linea verde
	for j in range(ny):
		if (img_ex[i,j,0]<=20) and (img_ex[i,j,2]<=20) and (img_ex[i,j,1]>=0): img_ex[i,j,:]=(0,0,0) 
_,img_ex = cv2.threshold(img_ex,127,255,cv2.THRESH_BINARY)
cv2.imshow('imagen_ejemplo',img_ex)
cv2.waitKey(0)
cv2.destroyAllWindows()

print('************************************************')
print('************************************************')
print('************************************************')
print("..... Importando Datos .....")
print('Tamano del lote ',batch_size)
print('Numero de datos de entrenamiento ',len(X_train))
print('Numero de datos de validacion ',len(X_val))
print('Tamaño de las entradas (1/2)',(nx,ny,ch)) # Suponiendo que las mascaras son imagenes de 3 canales
print('************************************************')
print('************************************************')
print('************************************************')

steps_per_epoch_val = 50 #len(X_train)/batch_size_val
validation_steps_val = 50 #len(X_val)/batch_size_val
data_gen_train = Data_Generator(X_train, steering_train, batch_size_val)
data_gen_val = Data_Generator(X_val, steering_val, batch_size_val)

model = build_CNN()
history=model.fit_generator(data_gen_train, steps_per_epoch=steps_per_epoch_val, epochs=epochs_val,validation_data=data_gen_val, validation_steps=validation_steps_val)

# Guarda el modelo entrenado
model.save('EE_learning_model.h5') 
# Grafica de la funcion de costo a lo largo de las epocas
plt.figure(1)
plt.plot(history.history['loss'])
plt.plot(history.history['val_loss'])
plt.legend(['training','validation'])
plt.title('loss')
plt.xlabel('epoch')
plt.grid()
# Grafica de la precision a lo largo de las epocas
plt.figure(2)
plt.plot(history.history['mean_absolute_error'])
plt.plot(history.history['val_mean_absolute_error'])
plt.legend(['training','validation'])
plt.title('Mean Absolute Error')
plt.xlabel('epoch')
plt.grid()
plt.show()
