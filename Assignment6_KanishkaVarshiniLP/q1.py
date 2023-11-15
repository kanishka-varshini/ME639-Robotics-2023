#Consider a planar 3R manipulator with link lengths 1 m each. Using the
# method that Shail discussed in class last week, write an inverse kinematics
# code to solve for the joint angles required to trace a circle of radius 1.5 m
# centred at the base of the manipulator. Verify your results by plotting the
# results and observing if it indeed traces the desired circle.


# planar 3R manipulator
# all rotations are about z (out of plane)

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


#rotation matrix about z axis
def Rzq(q):
    R=[[np.cos(q), -np.sin(q), 0],
       [np.sin(q), np.cos(q), 0],
       [0, 0, 1]]
    return R

# transformation matrix
def H(R, d):
    H1=[[R[0][0],R[0][1], R[0][2],d[0][0]],
       [R[1][0],R[1][1], R[1][2],d[1][0]],
       [R[2][0],R[2][1], R[2][2],d[2][0]],
       [0,0, 0,1]]
    return H1


def trajectory(t):
    x=1.5*np.sin(t)
    y=1.5*np.cos(t)
    return x,y,0

def jacobian_3R(q,l):
    q1,q2,q3=q[0],q[1],q[2] #q1,q2,q3 qre relative angles

    J=[
    [-l1*np.sin(q1)-l2*np.sin(q1+q2)-l3*np.sin(q1+q2+q3),-l2*np.sin(q1+q2)-l3*np.sin(q1+q2+q3),-l3*np.sin(q1+q2+q3)],
    [l1*np.cos(q1)+l2*np.cos(q1+q2)+l3*np.cos(q1+q2+q3), l2*np.cos(q1+q2)+l3*np.cos(q1+q2+q3), l3*np.cos(q1+q2+q3)],
    [1,1,1]
    ]
    return J

def pseudoinv(A):
    A_trans=np.transpose(A)
    A_inv=np.matmul(A_trans,np.linalg.inv(np.dot(A,A_trans)))

    return A_inv



def inv_kinematics(q_k1, x_k, x_k1, dt):
    J = jacobian_3R(q_k1, l)
    Z=pseudoinv(J)
    if np.linalg.norm(Z)>0.05:
        print("singularity")
        return
    v_k = np.array([(x_k[i] - x_k1[i]) * dt for i in range(len(x_k))])
    
    # Use np.dot for matrix multiplication instead of np.linalg.multi_dot
    q_k = q_k1 + np.dot(Z, v_k)*dt

    x_k1 = x_k  # updating value for the next iteration
    q_k1 = q_k
    return q_k


# using forward kinematics to compute the position of the end effector
def endeff_3R(q):
    q1,q2,q3=q[0],q[1],q[2] #q1,q2,q3 qre relative angles
     
    #for a 3R planar manipulator:

    H01=H(Rzq(q1), [[l1],[0],[0]])
    H12=H(Rzq(q2), [[l2],[0],[0]])
    H23=H(Rzq(q3), [[l3],[0],[0]])
    P=np.linalg.multi_dot([H01,H12,H23,[[0],[0],[0],[1]]])

    return P[0],P[1]


# link lengths
l=[1,1,1]
l1,l2,l3=l[0],l[1],l[2]
q_0=[np.pi/2,0,0] #initial joint angles
x_0=[1.5,0,0] #initial end effector position

J=jacobian_3R(q_0,l)

x_k1=x_0
q_k1=q_0

rads=np.arange(0,3*np.pi,0.1)
ref_x=1.5*np.sin(rads)
ref_y=1.5*np.cos(rads)

# q_k=inv_kinematics(q_k1,trajectory(3.14),x_0,0.1)
# x_=endeff_3R(q_k)
# print(x_)


# animating
fig, ax = plt.subplots(figsize=(6, 6))   
ax.set_xlim(-5, 5 )
ax.set_ylim(-5, 5 )
line, = ax.plot([], [], 'o-', lw=2)
line2, = ax.plot([], [], 'o-', lw=2)
line3, = ax.plot([], [], 'o-', lw=2)
# line4, = ax.plot([], [], 'o-', lw=2)


# time parameters
t_max = 20   # total simulation time (s)
num_steps =2000
dt=0.01

for i in range(0,num_steps):
    t=i*0.01
    x_k=trajectory(t) #end effector

    q=inv_kinematics(q_k1,x_k,x_k1,dt) 
    q1,q2,q3=q[0],q[1],q[2]

    # x,y=endeff_3R(q)
    x1,y1=l1*np.cos(q1),l1*np.sin(q1)
    x2,y2=l1*np.cos(q1)+l2*np.cos(q1+q2),   l1*np.sin(q1)+l2*np.sin(q1+q2)
    x3,y3=l1*np.cos(q1)+l2*np.cos(q1+q2)+l3*np.cos(q1+q2+q3),     l1*np.sin(q1)+l2*np.sin(q1+q2)+l3*np.cos(q1+q2+q3)

    line.set_data([0, x1], [0, y1])
    line2.set_data([x1,x2],[y1,y2])
    line3.set_data([x2,x3],[y2,y3])
    plt.pause(0.01) 

# plt.show()
    



# def init():
#     line.set_data([], [])
#     line2.set_data([],[])
#     line3.set_data([],[])
#     # line4.set_data([],[])
#     return line, line2,line3

# def animate(i):
#     t=i*0.01
#     x_k=trajectory(t) #end effector

#     q=inv_kinematics(q_k1,x_k,x_k1,dt) 
#     q1,q2,q3=q[0],q[1],q[2]

#     # x,y=endeff_3R(q)
#     x1,y1=l1*np.cos(q1),l1*np.sin(q1)
#     x2,y2=l1*np.cos(q1)+l2*np.cos(q1+q2),   l1*np.sin(q1)+l2*np.sin(q1+q2)
#     x3,y3=l1*np.cos(q1)+l2*np.cos(q1+q2)+l3*np.cos(q1+q2+q3),     l1*np.sin(q1)+l2*np.sin(q1+q2)+l3*np.cos(q1+q2+q3)


#     line.set_data([0, x1], [0, y1])
#     line2.set_data([x1,x2],[y1,y2])
#     line3.set_data([x2,x3],[y2,y3])
#     plt.pause(0.01) 
#     plt.show()
#     # line4.set_data(ref_x[0:i],ref_y[0:i])
#     # return line, line2, line3



# # ani = FuncAnimation(fig, animate, frames=num_steps, init_func=init, interval=(0.01) * 1000)


# # #plotting
# # plt.xlabel('x')
# # plt.ylabel('y')
# # plt.title('planar 3R manipulator tracing a circle of 1.5 radius')
# # plt.grid()
# # plt.show()