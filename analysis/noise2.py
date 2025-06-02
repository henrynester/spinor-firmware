import numpy as np
import matplotlib.pyplot as plt

table = np.genfromtxt('noise2.csv', delimiter=',')[1:1000,(0,1,3,5,7)]  
t = table[:,0] - table[0,0]
ia = table[:,1]
ib = table[:,2]
ic = table[:,3]
theta_e=table[:,4]
theta_e=theta_e*2*np.pi/2**14
omega_e = np.concatenate(((0,), (theta_e[1:]-theta_e[:-1])))
omega_e += (omega_e < 0) * 2*np.pi

igamma=(ia+ib+ic)/3
ia=igamma-ia
ib=igamma-ib
ic=igamma-ic

ialpha=ia
ibeta=1/np.sqrt(3) * ia + 2/np.sqrt(3) * ib

i_d=np.cos(theta_e)*ialpha+np.sin(theta_e)*ibeta
i_q=-np.sin(theta_e)*ialpha+np.cos(theta_e)*ibeta

#i_d_filt = np.convolve(np.ones(10)/10, i_d, mode='same')
#i_q_filt = np.convolve(np.ones(10)/10,i_q,mode='same')

fig,axs=plt.subplots(4)
axs[0].plot(t, ia, 'r')
axs[0].plot(t, ib, 'g')
axs[0].plot(t, ic, 'b')
axs[1].plot(t, ialpha, 'r')
axs[1].plot(t, ibeta, 'b')
axs[2].plot(t, i_d, 'r')
axs[2].plot(t, i_q, 'b')
axs[2].twinx().plot(t, theta_e, 'gray')
axs[3].plot(t, theta_e)
axs[3].plot(t, omega_e)
plt.show()

t=np.linspace(0,2* 2*np.pi,1000)
y=np.sin(2*np.pi*t) + 0.5*np.sin(2*np.pi*2*t+np.pi/2)
fig, ax = plt.subplots()
ax.plot(t,y)
plt.show()
