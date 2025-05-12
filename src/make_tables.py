import numpy as np
import matplotlib.pyplot as plt

#sin/cos table, single quadrant only:
lsb_theta_e = 2*np.pi / (2**14)
N_downsample_sin = 2**4
lsb_sin_arg = lsb_theta_e * N_downsample_sin 
lsb_sin = 1 / (2**15-1)

sin_arg_rep = np.arange(0, 2**14//N_downsample_sin //4)
sin_arg = sin_arg_rep * lsb_sin_arg
sin = np.sin(sin_arg)
sin_rep = (sin / lsb_sin).astype(np.int16) 

#fig,ax=plt.subplots()
#ax.plot(sin_arg_rep, sin_rep, marker='.')
#plt.show()

macro = '#define SIN_LUT {'
for i,_sin_rep in enumerate(sin_rep):
    if i % 10 == 0:
        macro += '\\\n'
    macro += f'{_sin_rep:6},'
macro += f'{32767:6}}}'
print(macro)
print()

#thermistor table
T_degC=np.linspace(0,100,11)
T=T_degC+273
B=3900 #K
T0=273+25 #K
R0=10e3 #Ohms, at T0 
R=R0*np.exp(B*(1/T-1/T0))
Ru=4.7e3
adc=(4096*R/(R+Ru)).astype('int')
#fig,ax=plt.subplots()
#ax.plot(T_degC,adc,marker='.')
#plt.show()

macro = '#define NTC_LUT {'
for i,_adc in enumerate(adc):
    if i % 5 == 0:
        macro += '\\\n'
    macro += f'{_adc:6}'
    if i<len(adc)-1:
        macro += ','
macro += '}'
print(macro)
