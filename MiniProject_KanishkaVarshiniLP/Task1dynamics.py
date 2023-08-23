import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
from matplotlib.animation import FuncAnimation

# defining the constants

l1=2.0 # length of link 1
l2=1.0 # length of link 2

m1=1.0 # mass of link 1
m2=1.0 # mass of link 2

q1=0 # angle between link 1 and horizontal
q2=0 # angle between link 2 and horizontal

# task 1: given arbitrary trajectory of the end effector, make the robot follow the trajectory

# using forward kinematics to compute the position of the end effector
def forward_kinematics(q1,q2):
    x = l1*np.cos(q1) + l2*np.cos(q2)
    y = l1*np.sin(q1) + l2*np.sin(q2)

    return x, y # end effector coordinates


# inverse kinematics
def inv(x,y):
    theta=np.arccos((x**2+y**2-l1**2-l2**2)/(2*l1*l2)) # in radians
    q1= np.arctan2(y, x) - np.arctan2(l2 * np.sin(theta), l1 + l2 * np.cos(theta)) #in radians
    q2= q1+theta
    return q1,q2


xd,yd=1,2 #desired position
q1d,q2d=inv(xd,yd)
x0,y0=3,0 #initial position


# dynamics
def torques(state,t):

    kp=5 # control gain
  
    q1,q2=state[0],state[1]
    q1_dot,q2_dot=state[2],state[3]

    x,y=forward_kinematics(q1,q2) # current position of end effector

    T1=kp*(q1d-q1)-5*q1_dot # the larger the angular velocity, the less easy to stop at desired location, therefore it should reduce the torque
    T2=kp*(q2d-q2)-5*q2_dot-T1 # T1 imparts momentum to joint O2

    return T1,T2
    


def q_dotdot(state,t):

    q1,q2,q1_dot,q2_dot=state
    
    T1,T2=torques(state,t)
    lhs=[[(m1*l1*2)/3 + m2*l1*2, 0.5*m2*l1*l2*np.cos(q2-q1)],[0.5*m2*l1*l2*np.cos(q2-q1), (m2*l2*2)/3 + 0.25*m2*l2*2]]
    rhs=[[T1+0.5*l1*l2*q2_dot*(q2_dot-q1_dot)*np.sin(q2-q1)], [T2+0.5*l1*l2*q1_dot*(q2_dot-q1_dot)*np.sin(q2-q1)]]

    [q1_dotdot,q2_dotdot]=np.matmul(np.linalg.inv(lhs), rhs)
    
    return [q1_dot, q2_dot,q1_dotdot[0],q2_dotdot[0]]



# initial conditions
q1_dot,q2_dot=0,0
u0=[0,0,q1_dot,q2_dot]


# time parameters
t_max = 100   # total simulation time (s)
num_steps = 500
time = np.linspace(0, t_max, num_steps)


#integrate using odeint

states = odeint(q_dotdot, u0, time)


# animate the motion of the robot

fig, ax = plt.subplots(figsize=(6, 6))   
ax.set_xlim(-1.5 * l1, 1.5 * l1)
ax.set_ylim(-1.5 * l1, 1.5 * l1)
line, = ax.plot([], [], 'o-', lw=2)
line2, = ax.plot([], [], 'o-', lw=2)

def init():
    line.set_data([], [])
    line2.set_data([],[])

    return line, line2,

def animate(i):
    t=i*t_max/num_steps

    q1=states[i,0]
    q2=states[i,1]

    x,y=forward_kinematics(q1,q2)
    x1=l1*np.cos(q1)
    y1=l1*np.sin(q1)
 
    line.set_data([0, x1], [0, y1])
    line2.set_data([x1,x],[y1,y])
    return line, line2,


ani = FuncAnimation(fig, animate, frames=num_steps, init_func=init, blit=True, interval=(t_max / num_steps) * 1000)




#plotting
plt.xlabel('x')
plt.ylabel('y')
plt.title('task 1: dynamics considered')
plt.grid()
plt.show()
