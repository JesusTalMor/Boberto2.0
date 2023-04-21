import numpy as np
# pos inicial del robot
u = np.zeros((3,1))
# cov matrix inicial
sigma = np.zeros((3,3))
#  motion model cov matrix
Qk = np.array([[0.5,0.01,0.01],
                [0.01,0.5,0.01],
                [0.01,0.01,0.2]])
# constants
v = 1
w = 1
dt = 0.1
#while(True):
for i in range(2):
    u0 = u
    u[0] = u0[0] + (dt * v * np.cos(u[2]))
    u[1] = u0[1] + (dt * v * np.sin(u[2]))
    u[2] = u0[2] + (dt*w)
    H = np.array([[1,0,(-dt*v*np.sin(u0[2]))],
                  [0,1,(dt*v*np.cos(u0[2]))],
                  [0,0,1]])
    sigma = H.dot(sigma).dot(H.T) + Qk
    print ("H {}".format(i+1))
    print (H)