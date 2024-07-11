import numpy as np
import matplotlib.pyplot as plt
import math

center_line = np.loadtxt('center_line.txt',delimiter=',')
print(center_line)
# print(np.stack((center_line,center_line[:1,:]),axis=0)[1:].shape)
diffs = np.concatenate((center_line,center_line[:1,:]),axis=0)[1:] - np.concatenate((center_line[-1:,:],center_line),axis=0)[:-1]
yaws = np.arctan2(diffs[:,1],diffs[:,0])
left_line = [center_line[:,0] + center_line[:,2]/2.*np.cos(yaws+math.pi/2.),center_line[:,1] + center_line[:,2]/2.*np.sin(yaws+math.pi/2.)]
left_line = np.array(left_line).T
right_line = [center_line[:,0] + center_line[:,2]/2.*np.cos(yaws-math.pi/2.),center_line[:,1] + center_line[:,2]/2.*np.sin(yaws-math.pi/2.)]
right_line = np.array(right_line).T
left_line = np.concatenate((left_line,left_line[:1,:]),axis=0)
right_line = np.concatenate((right_line,right_line[:1,:]),axis=0)
center_line = np.concatenate((center_line,center_line[:1,:]),axis=0)
plt.plot(center_line[:,0],center_line[:,1],label='Center line')
plt.plot(left_line[:,0],left_line[:,1],label='Left boundary')
plt.plot(right_line[:,0],right_line[:,1],label='Right boundary')
plt.axis('equal')
plt.legend()
plt.show()
