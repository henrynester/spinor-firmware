import numpy as np
import matplotlib.pyplot as plt

theta_in_theta_out = np.genfromtxt('cal001.csv', delimiter=',')[1:,][:,(1,3)]  
theta_in = theta_in_theta_out[:,0]
theta_out = theta_in_theta_out[:,1]
err = (theta_out - theta_in) % (2*14)
err_avg=np.mean(err)
err-=err_avg
print(err_avg)

err_fwd=err[:len(err)//2]
err_rev=np.flip(err[len(err)//2:])

err_decog = (err_fwd+err_rev)/2
err_filt = np.convolve(err_decog, np.ones(146)/146, mode='same')
#err -= err_avg

fig,ax=plt.subplots()
ax.plot(err_decog)
ax.plot(err_filt)
plt.show()
