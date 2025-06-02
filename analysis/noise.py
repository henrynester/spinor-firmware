import numpy as np
import matplotlib.pyplot as plt

table = np.genfromtxt('noise.csv', delimiter=',')[1:1000,(0,1,3,5)]  
t = table[:,0] - table[0,0]
ia = -table[:,1]
ib = -table[:,2]
ic = -table[:,3]

igamma=(ia+ib+ic)/3

ia-=igamma
ib-=igamma
ic-=igamma

ialpha = ia
ibeta = (1/np.sqrt(3))*ia+(2/np.sqrt(3))*ib
mag = np.sqrt(ialpha**2+ibeta**2)
Navg=10
magfilt = np.convolve(mag, np.ones(Navg)/Navg, mode='same')

navg=10
#i = np.sin(2*np.pi*10*t)

#fft_i=fft_i[:len(fft_i)//2]

##N = len(fft_i)
#n = np.arange(N)
#T = N*np.mean(t[1:]-t[:-1])
#freq = n/2/T

fig,axs=plt.subplots(3)
axs[0].plot(t, ia, 'r')
axs[0].plot(t, ib, 'g')
axs[0].plot(t,ic,'b')
axs[1].plot(t,ialpha,'r')
axs[1].plot(t,ibeta,'b')
axs[2].plot(t,mag)
axs[2].plot(t,magfilt)
#axs[1].plot(freq, np.log10(fft_i))
plt.show()
