import numpy as np
import matplotlib.pyplot as plt
import math
import matplotlib

matplotlib.rcParams.update({'font.size': 14})
Df = 15.
Cf = 1.5
Bf = 4.


ts = np.arange(0.,1.09,0.2)
i = 0
for t in ts :
    Dfn = Df*3**(1-t)
    Cfn = Cf/(Cf**(2-2*t))
    Bfn = 2**(1-t)*Df*Cf*Bf/(Dfn*Cfn)
    theta = np.arange(0.,0.5,0.001)
    Fs = Dfn*np.sin(Cfn*np.arctan(Bfn*theta))
    if i < 10 :
        plt.plot(theta,Fs,label='t=0.'+str(i))
    else :
        plt.plot(theta,Fs,label='t=1.'+str(i-10))
    i += 2
plt.xlabel('Slip angle [rad]')
plt.ylabel('Tire lateral force [N]')
plt.legend()
plt.show()