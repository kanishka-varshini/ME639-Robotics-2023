import numpy as np

#SCARA RRP configuration
#All rotations are about z axis and all the z axes (in different frames) are parallel.

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

# getting the joint variables

l1=float(input("length of first link: "))
l2=float(input("length of second link: "))
l3=float(input("length of third link: "))

q1=float(input("the angular displacement of the first joint: "))
q2=float(input("the angular displacement of the second joint: "))
q3=float(input("the linear displacement of the prismatic joint: "))

q1=np.radians(q1)
q2=np.radians(q2)

H01=H(Rzq(q1), [[0],[0],[l1]])
H12=H(Rzq(q2), [[0],[l2],[0]])
H23=H(Rzq(0), [[0],[0],[-q3]])
P=np.linalg.multi_dot([H01,H12,H23,[[0],[0],[-l3],[1]]])

print("\nfinal position of the end effector with respect to the base frame : ", P[0][0],"i +",P[1][0],"j +", P[2][0],"k")
