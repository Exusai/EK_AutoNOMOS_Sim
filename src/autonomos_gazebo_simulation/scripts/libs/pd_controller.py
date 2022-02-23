#! /usr/bin/env python
import numpy as np

def pd_controller(x1, x2, x_ref, l, ey_h):
	# CONTROL
	ky = 0.0909961 
	kth = 0.20056981
	kdy = 0.0075
	h_vis =1.0/30.0
	# vrpm	R			Q				Ky					Kth					Kdy
	# 800		24		0.04*I	0.0909961 , 0.20056981, 0.0075  +++
	e_y = x1-x_ref
	e_th = np.arctan2(x2-x1,l)
	de_y = (e_y-ey_h)/h_vis
	u = int(round(90-np.arctan(ky*e_y+kth*e_th+kdy*de_y)*(180/np.pi))) 
	return e_y, e_th, u
