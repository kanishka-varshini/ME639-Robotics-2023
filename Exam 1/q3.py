#Kanishka Varshini L P
#21110094

import numpy as np

#6R configuration
#All rotations are about the current z axes.

# rotation matrix about z axis
def Rzq(q):
    R=[[np.cos(q), -np.sin(q), 0],
       [np.sin(q), np.cos(q), 0],
       [0, 0, 1]]
    return R

# rotation matrix about y axis
def Ryq(q):
    R=[[np.cos(q), 0, np.sin(q)],
       [0, 1, 0],
       [-np.sin(q), 0, np.cos(q)]]
    return R

# rotation matrix about x axis
def Rxq(q):
    R=[[1, 0, 0],
       [0, np.cos(q), -np.sin(q)],
       [0, np.sin(q), np.cos(q)]]
    return R

# transformation matrix
def H(R, d):
    H1=[[R[0][0],R[0][1], R[0][2],d[0][0]],
       [R[1][0],R[1][1], R[1][2],d[1][0]],
       [R[2][0],R[2][1], R[2][2],d[2][0]],
       [0,0, 0,1]]
    return H1

# taking the joint variables from input

a1=float(input("offset length a1: "))
l2=float(input("length of link l2: "))
l3=float(input("length of link l3: "))
a4=float(input("offset length a4: "))
a5=float(input("offset length a5: "))
a6=float(input("offset length a6: "))
a7=float(input("offset length a7: "))

# joint angles
q1=np.radians(float(input("the angular displacement of the first joint: ")))
q2=np.radians(float(input("the angular displacement of the second joint: ")))
q3=np.radians(float(input("the angular displacement of the third joint: ")))
q4=np.radians(float(input("the angular displacement of the fourth joint: ")))
q5=np.radians(float(input("the angular displacement of the fifth joint: ")))
q6=np.radians(float(input("the angular displacement of the sixth joint: ")))


# transformation matrices
# refer diagram from answer 1 and answer 2
H01=H(Rzq(q1), [[0],[0],[a1]])
H12=H(np.matmul(Rxq(np.pi/2),Rzq(q2)), [[0],[0],[0]])
H23=H(Rzq(q3), [[-l2],[0],[0]])
H34=H(Rzq(q4),[[-l3],[0],[a4]])
H45=H(np.matmul(Rxq(-np.pi/2),Rzq(q5)),[[0],[-a5],[0]])
H56=H(np.matmul(Rxq(np.pi/2),Rzq(q6)),[[0],[a6],[0]])

P6=[[0],[0],[a7],[1]] # position of end effector wrt frame 6

#position of end effector in 0 frame
P=np.linalg.multi_dot([H01,H12,H23,H34,H45,H56,P6])

print("\nfinal position of the end effector with respect to the base frame : ", P[0][0],"i +",P[1][0],"j +", P[2][0],"k")