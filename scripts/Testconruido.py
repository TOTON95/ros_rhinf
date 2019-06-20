#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue May  7 10:00:01 2019

@author: Ignacio
"""

import numpy as np
import numpy.linalg as nla
import matplotlib.pyplot as plt

plt.close("all")

# funcion para saturar la entrada
def mysat(u):
    Sat=np.ones((u.shape))*1
    return np.sort(np.vstack((-Sat,u,Sat)),axis=0)[1]

An=np.array([[1,1],[0, 0.99851729]]) # dinamica de los estados
Bn=np.array([[0],[0.0002362]]) # donde entra la entrada en el sistema
Cn=np.array([[1,0]])   # Que estado es la salida

Bp=np.array([[0],[1]]) # Donde actua la pertubacion 

F=np.array([[  -3.52718328, -269.24932787]])   # Controlador base
Fn=np.array([[0.00140837,0.02207816]])         # Controlador robusto
Kdis=np.array([[0,-4233.78427452]])            # Rechazo de perturbacion 
#%%

Tf=4500 # tiempo de simulacion

# Generacion de la pertubacion rampa
p=np.zeros((Tf))
p[3200:3800]=np.arange(3200,3800)/600*0.000031

# Generacion de la referencia
ref=np.zeros((2,Tf))
ref[:,200:1000]=np.array(([[-1],[0]]))
ref[:,1700:2500]=np.array(([[4],[0]]))

Ven=50 # ancho de la ventana de optimizacion
a=0.5 #constante del filtro

vk=(np.random.rand(Tf)*2-1)*0.0002   #ruido

# Alocacion en la memoria
x=np.zeros((2,Tf))
xf=np.zeros((2,Tf))
x[:,[0]]=np.array([[0],[0]])
u=np.zeros((Tf))
y=np.zeros((Tf))
y[0]=Cn@x[:,[0]]
vf=np.zeros((Tf))
dis=np.zeros((2,Tf))

for i in range(Tf-1):
    xf[:,[i]]=np.vstack((y[i],vf[i])) # Estado con la posicion y la velocidad
    if i%17==0:
        e=xf[:,[i]]-ref[:,[i]]        # Calculo del error
        # Calculo de la perturbacion por minimos cuadrados
        aux=0
        for ii in range(i-Ven+1,i+1):
            aux=aux+xf[:,[ii]]-An@xf[:,[ii-1]]-Bn*mysat(u[ii-1])
        #endFor    
        dis[:,[i]]=aux/Ven
        
        u[i]=mysat(F@e-3.6386*nla.norm(np.exp(-1.1*np.abs(e))-1,2)*Fn@e+Kdis@dis[:,[i]]) # Ley de control
    else:
        dis[:,[i]]=dis[:,[i-1]]
        u[i]=u[i-1]
    #endIf    
    x[:,[i+1]]=An@x[:,[i]]+Bn*u[i]+Bp*p[i] # Simulacion del modelo
    y[i+1]=Cn@x[:,[i+1]]+vk[i]             # Salida del modelo con ruido
    
    vf[i+1]=(1-a)*vf[i]+a*(y[i]-y[i-1])    # Estimacion de la velocidad
#endFor
    
#Figuras
plt.figure(0)
plt.subplot(2,1,1)
plt.plot(np.arange(Tf)/100,x[[0],:].T)
plt.subplot(2,1,2)
plt.plot(np.arange(Tf)/100,x[[1],:].T)
plt.figure(1)
plt.plot(np.arange(Tf)/100,u)
plt.figure(3)
plt.subplot(2,1,1)
plt.plot(np.arange(Tf)/100,dis[0].T)
plt.plot(np.arange(Tf)/100,p.T*0)
plt.subplot(2,1,2)
plt.plot(np.arange(Tf)/100,dis[1].T)
plt.plot(np.arange(Tf)/100,p.T)
plt.figure(4)
plt.plot(np.arange(Tf)/100,y.T)
    
 
