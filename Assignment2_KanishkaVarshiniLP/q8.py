import numpy as np

# RRP SCARA configuration 
# Manipulator Jacobian matrix

l1=float(input("length of first link: "))
l2=float(input("length of second link: "))
l3=float(input("length of third link: "))

q1=np.radians(float(input("the angular displacement of the first joint: ")) )
q2=np.radians(float(input("the angular displacement of the second joint: ")) )
q3=float(input("the linear displacement of the prismatic joint: "))

J=[[-(l2*np.cos(q1)+l3*np.cos(q1+q2)),-l3*np.cos(q1+q2),0],
   [-(l2*np.sin(q1)+l3*np.sin(q1+q2)),-l3*np.sin(q1+q2),0],
   [0,0,0],
   [0,0,0],
   [0,0,0],
   [1,1,0]]

print(J)