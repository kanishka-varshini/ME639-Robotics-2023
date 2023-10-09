import numpy as np

#D-H parameters d,theta,a,alpha
#All rotations are about the current z axes.


l=11,11.5 # link lengths

# rotation matrix about z axis
def Rzq(q_):
    R=[[np.cos(q_), -np.sin(q_), 0],
       [np.sin(q_), np.cos(q_), 0],
       [0, 0, 1]]
    return R

# rotation matrix about y axis
def Ryq(q):
    R=[[np.cos(q), 0, np.sin(q)],
       [0, 1, 0],
       [-np.sin(q), 0, np.cos(q)]]
    return R

# rotation matrix about x axis
def Rxq(q_):
    R=[[1, 0, 0],
       [0, np.cos(q_), -np.sin(q_)],
       [0, np.sin(q_), np.cos(q_)]]
    return R

# transformation matrix
def H(R=np.identity(3), d=[0,0,0]): #default no rotation, no translation
    H1=[[R[0][0],R[0][1], R[0][2],d[0]],
       [R[1][0],R[1][1], R[1][2],d[1]],
       [R[2][0],R[2][1], R[2][2],d[2]],
       [0,0, 0,1]]
    return H1

def A(d,theta,a,al):
    d1=[0,0,d]
    a1=[a,0,0]

    H1=H(np.identity(3),d1)
    H2= H(Rzq(theta))
    H3=H(np.identity(3),a1)
    H4=H(Rxq(al),[0,0,0])
    Hab=(np.linalg.multi_dot([
                H(np.identity(3),d1),
                H(Rzq(theta)),
                 H(np.identity(3),a1),
                 H(Rxq(al),[0,0,0])
                 ]))
    # Ai=[
    #     [np.cos(theta),-np.sin(theta)*np.cos(al),np.sin(theta)*np.sin(al),a*np.cos(theta)],
    #     [np.sin(theta),np.cos(theta)*np.cos(al),-np.cos(theta)*np.sin(al),a*np.sin(theta)],
    #     [0,np.sin(al),np.cos(al),d],
    #     [0,0,0,1]
    # ]
    return Hab


def endeffector(dh):   # forward kinematics
    # transformation matrices
    H1=[]
    for i in dh:
        H1+=[A(i[0],i[1],i[2],i[3])]
    H_=H1[0]
    for j in range(1,len(H1)):
        H_=np.matmul(H_,H1[j])
    
    #position of end effector in 0 frame
    P=np.matmul([H_,[0,0,0,1]])

    #print("final position of the end effector with respect to the base frame : ", P[0],"i +",P[1],"j +", P[2],"k")
    return [P[0],P[1],P[2]]

# def jacobian(config):

#     return 0


import sympy as sp

def jacobian(dh):
    thetas = sp.symbols('theta:' + str(len(dh)))
    J = sp.Matrix([[0] * len(dh) for _ in range(6)])
    T = sp.eye(4)
    R = sp.eye(3)

    for i in range(len(dh)):
        di, theta, ai, alpha= dh[i]

        A = sp.Matrix([
            [sp.cos(theta), -sp.sin(theta) * sp.cos(alpha), sp.sin(theta) * sp.sin(alpha), ai * sp.cos(theta)],
            [sp.sin(theta), sp.cos(theta) * sp.cos(alpha), -sp.cos(theta) * sp.sin(alpha), ai * sp.sin(theta)],
            [0, sp.sin(alpha), sp.cos(alpha), di],
            [0, 0, 0, 1]
        ])
        # A_=A(dh[i][0],dh[i][1],dh[i][2],dh[i][3])
        T = T * A
        R = R * A[:3, :3]

        if theta == 1:
            k = R * sp.Matrix([0, 0, 1])
            J[3, i] = k[0]
            J[4, i] = k[1]
            J[5, i] = k[2]

    for i in range(len(dh)):
        J[0, i] = T[0, 3].diff(thetas[i])
        J[1, i] = T[1, 3].diff(thetas[i])
        J[2, i] = T[2, 3].diff(thetas[i])

    return J

# Example DH parameters for a 2R manipulator:
# theta1, theta2 = sp.symbols('theta1 theta2') # modify this to include all variable angles and distances.
# dh_parameters = [
#     [0, theta1, 1, 0],
#     [0, theta2, 1, 0]
# ]

# jacobian = jacobian(dh_parameters)
# jacobian.simplify()
# jacobian = jacobian.tolist()


n=int(input('enter the number of joints: '))
config=input('enter a string of the configuration of the joints, e.g.RRP: ')
dh=eval(input('enter the D-H parameters, e.g.[[0,q1,l1,0],[0,q2,l2,0]]: '))

P=endeffector(dh)

jacobian_= jacobian(dh)
jacobian_.simplify()
jacobian_= jacobian.tolist()
jacobian_=jacobian_.transpose()

endeff_vel=np.matmul(jacobian_[0],P)

print("(a)manipulator jacobian: \n", jacobian_)
print("(b)end effector position: ",P)
print("(b)end effector velocity: ",P)