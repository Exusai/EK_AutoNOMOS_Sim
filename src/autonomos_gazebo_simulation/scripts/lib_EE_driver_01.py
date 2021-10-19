import numpy as np
from model_builder import nn_Dlayer, nn_Clayer, nn_Pool, nn_flatt

#************************************************************************
#************************************************************************
#************************************************************************
def ee_driver(B_c,W_c,B_d,W_d,imagenR):
	C1 = nn_Clayer(imagenR,B_c[0],W_c[0],'relu',padding=0,stride=[1,1])
	P1 = nn_Pool(C1,2, m='max_P')
	C2 = nn_Clayer(P1,B_c[1],W_c[1],'relu',padding=0,stride=[1,1])
	P2 = nn_Pool(C2,2, m='max_P')
	C3 = nn_Clayer(P2,B_c[2],W_c[2],'relu',padding=0,stride=[1,1])
	P3 = nn_Pool(C3,5, m='max_P')
	F = nn_flatt(P3)
	D1 = nn_Dlayer(F,B_d[0],W_d[0],'relu')
	D2 = nn_Dlayer(D1,B_d[1],W_d[1],'relu')
	y = nn_Dlayer(D2,B_d[2],W_d[2],'none')
	return y

