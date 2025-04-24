import numpy as np
import matplotlib.pyplot as plt

lsb_theta_e = 2*np.pi / (2**14)
N_downsample_sin = 2**6
lsb_sin_arg = lsb_theta_e * N_downsample_sin 
lsb_sin = 1 / (2**15-1)

sin_arg_rep = np.arange(0, 2**14//N_downsample_sin //4)
sin_arg = sin_arg_rep * lsb_sin_arg
sin = np.sin(sin_arg)
sin_rep = (sin / lsb_sin).astype(np.int16) 

fig,ax=plt.subplots()
ax.plot(sin_arg_rep, sin_rep, marker='.')
#plt.show()

macro = '#define SIN_LUT {'
for i,_sin_rep in enumerate(sin_rep):
    if i % 10 == 0:
        macro += '\\\n'
    macro += f'{_sin_rep:6}'
    if i < len(sin_rep) - 1:
        macro += ','
macro += '}'
print(macro)
