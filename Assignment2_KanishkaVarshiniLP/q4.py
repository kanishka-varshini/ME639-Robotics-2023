import numpy as np

#Stanford-type RRP configuration
#The axes of rotation are perpendicular.

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

# rotation matrix about z axis
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

# getting the joint variables

l1=float(input("length of first link: "))
l2=float(input("length of second link: "))
l3=float(input("length of third link: "))

q1=np.radians(float(input("the angular displacement of the first joint: ")) )
q2=np.radians(float(input("the angular displacement of the second joint: ")))
q3=float(input("the linear displacement of the prismatic joint: "))


R01=Rzq(q1)
R12=np.matmul(Ryq(np.pi/2),Rzq(q2))
R23=np.identity(3)


H01=H(R01, [[0],[0],[0]])
H12=H(R12, [[0],[0],[l1]])
H23=H(R23, [[l2+q3],[0],[0]])

P=np.linalg.multi_dot([H01,H12,H23,[[l3],[0],[0],[1]]])

print("\nfinal position of the end effector with respect to the base frame : ", P[0][0],"i +",P[1][0],"j +", P[2][0],"k")