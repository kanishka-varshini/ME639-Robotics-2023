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


def endeffector(q,l):
    # transformation matrices
    # refer diagram from answer 1 and answer 2
    l=[a1,l2,a4,a5,a6,a7]
    H01=H(Rzq(q[0]), [[0],[0],[l[0]]])
    H12=H(np.matmul(Rxq(np.pi/2),Rzq(q[1])), [[0],[0],[0]])
    H23=H(Rzq(q[2]), [[-l[1]],[0],[0]])
    H34=H(Rzq(q[3]),[[-l[2]],[0],[l[3]]])
    H45=H(np.matmul(Rxq(np.pi/2),Rzq(q[4])),[[0],[-l[4]],[0]])
    H56=H(np.matmul(Rxq(-np.pi/2),Rzq(q[5])),[[0],[l[5]],[0]])

    P6=[[0],[0],[a7],[1]] # position of end effector wrt frame 6

    #position of end effector in 0 frame
    P=np.linalg.multi_dot([H01,H12,H23,H34,H45,H56,P6])

    print("\nfinal position of the end effector with respect to the base frame : ", P[0][0],"i +",P[1][0],"j +", P[2][0],"k")



# choosing the joint variables

a1=1 #offset length
l2=1 #length of link l2
l3=1 #length of link l3
a4=1 #offset length
a5=1 #offset length
a6=1 #offset length 
a7=1 #offset length
l=[a1,l2,l3,a4,a5,a6]

# joint angles
q=[0,0,0,0,0,0] #q1,q2,q3,q4,q5,q6

endeffector(q,l)
