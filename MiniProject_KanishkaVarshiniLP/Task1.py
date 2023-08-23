import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# defining the constants

l1=2.0 # length of link 1
l2=1.0 # length of link 2

m1=1.0 # mass of link 1
m2=1.0 # mass of link 2

q1=0 # angle between link 1 and horizontal
q2=0 # angle between link 2 and horizontal

# task 1: given arbitrary trajectory of the end effector, make the robot follow the trajectory



# dynamics not included

# using forward kinematics to compute the position of the end effector
def forward_kinematics(q1,q2):
    x = l1*np.cos(q1) + l2*np.cos(q2)
    y = l1*np.sin(q1) + l2*np.sin(q2)

    return x, y # end effector coordinates



# trajectory of end effector
def trajectory(t):
    #x = 2*np.cos(t)
    #y = 2*np.sin(t)
    x=2*t+1
    y=2*t+1
    return x, y


x,y=trajectory(0)



# inverse kinematics
def inv(x,y):
    cos_theta= (x**2+y**2-l1**2-l2**2)/(2*l1*l2)
    if cos_theta<-1 or cos_theta>1:
        exit(0)
    theta=np.arccos(cos_theta) # in radians
    q1= np.arctan2(y, x) - np.arctan2(l2 * np.sin(theta), l1 + l2 * np.cos(theta)) #in radians
    q2= q1+theta
    return q1,q2



# animating
fig, ax = plt.subplots(figsize=(6, 6))   
ax.set_xlim(-1.5 * l1, 1.5 * l1)
ax.set_ylim(-1.5 * l1, 1.5 * l1)
line, = ax.plot([], [], 'o-', lw=2)
line2, = ax.plot([], [], 'o-', lw=2)



# time parameters
t_max = 2*np.pi    # total simulation time (s)
num_steps = 500



def init():
    line.set_data([], [])
    line2.set_data([],[])
    return line, line2,

def animate(i):
    t=i*t_max/num_steps
    x,y=trajectory(t) #end effector


    q1,q2=inv(x,y)
    x1,y1=l1*np.cos(q1),l1*np.sin(q1) #O2

    x,y=forward_kinematics(q1,q2)

    line.set_data([0, x1], [0, y1])
    line2.set_data([x1,x],[y1,y])
    return line, line2,



ani = FuncAnimation(fig, animate, frames=num_steps, init_func=init, blit=True, interval=(t_max / num_steps) * 1000)


#plotting
plt.xlabel('x')
plt.ylabel('y')
plt.title('task 1: dynamics not considered')
plt.grid()
plt.show()

