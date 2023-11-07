import numpy as np

#Stanford manipulator.
#RRP configuration

# rotation matrix about z axis
def Rzq(q_):
    R=[[np.cos(q_), -np.sin(q_), 0],
       [np.sin(q_), np.cos(q_), 0],
       [0, 0, 1]]
    return R


# transformation matrix
def H(R=np.identity(3), d=[0,0,0]): #default no rotation, no translation
    H1=[[R[0][0],R[0][1], R[0][2],d[0]],
       [R[1][0],R[1][1], R[1][2],d[1]],
       [R[2][0],R[2][1], R[2][2],d[2]],
       [0,0, 0,1]]

    return H1

def A(d,theta,a,al):
    Ai=[
        [np.cos(theta),-np.sin(theta)*np.cos(al),np.sin(theta)*np.sin(al),a*np.cos(theta)],
        [np.sin(theta),np.cos(theta)*np.cos(al),-np.cos(theta)*np.sin(al),a*np.sin(theta)],
        [0,np.sin(al),np.cos(al),d],
        [0,0,0,1]
    ]
    return Ai

def stanford_endeff(q,l):   # forward kinematics
    q1,q2,q3=q[0],q[1],q[2]   #absolute angles
    
    # transformation matrices
    H01=H(Rzq(q1), [0,0,0])
    H12=H(np.matmul([[1, 0, 0],[0, 0, -1],[0, 1, 0]], Rzq(q2)), [0,0,l1])
    H23=H(np.identity(3), [l2+l3,0,0])
 
    #position of end effector in 0 frame
    P=np.linalg.multi_dot([H01,H12,H23,[[q3],[0],[0],[1]]])

    return P[0][0],P[1][0],P[2][0]


def stanford_inv(P,l):
    x,y,z=P[0],P[1],P[2]
    l1,l2,l3=l[0],l[1],l[2]

    s=z-l1
    r=np.sqrt(x**2+y**2)

    q1=np.arctan2(x,y)
    q2=np.arctan2(s,r)
    q3=np.sqrt(s**2+r**2)-l1-l2
    # print(q3)
    # print("joint variables for given end effector coordinates :",q1,q2,q3)
    return q1,q2,q3
    


# examples
P1=[0,0,3]
P2=[0,0,1]
P3=[1,1,1]
P=[P1,P2,P3]

l=[1,1,1]  #link lengths
l1,l2,l3=l[0],l[1],l[2]

for i in range(3):
    q=stanford_inv(P[i],l)
    P_=stanford_endeff(q,l)
    print("end eff position:",P[i],"\tinv kinematics joint variables(angles in radians):",q,"position calculated using forward kinematics:",P_)

