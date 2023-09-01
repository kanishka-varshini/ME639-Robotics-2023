import numpy as np

# RRR configuration
# Manipulator Jacobian matrix

l1=float(input("length of first link: "))
l2=float(input("length of second link: "))
l3=float(input("length of third link: "))

q1=np.radians(float(input("the angular displacement of the first joint: ")) )
q2=np.radians(float(input("the angular displacement of the second joint: ")) )
q3=np.radians(float(input("the angular displacement of the third joint: ")) )

J=[[-(l1*np.sin(q1)+l2*np.sin(q1+q2)+l3*np.sin(q1+q2+q3)),-(l2*np.sin(q1+q2)+l3*np.sin(q1+q2+q3)),-l3*np.sin(q1+q2+q3)],
   [(l1*np.cos(q1)+l2*np.cos(q1+q2)+l3*np.cos(q1+q2+q3)),(l2*np.cos(q1+q2)+l3*np.cos(q1+q2+q3)),l3*np.cos(q1+q2+q3)],
   [0,0,0],
   [0,0,0],
   [0,0,0],
   [1,1,1]]

print(J)