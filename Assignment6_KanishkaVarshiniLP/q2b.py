import numpy as np

# PUMA RRR configuration 
l1, l2, l3 = 0.25, 0.25, 0.25

def puma_endeff(q1, q2, q3):   

    #transformation matrices
    A1 = np.array([
        [np.cos(q1), -np.sin(q1), 0, 0],
        [np.sin(q1), np.cos(q1), 0, 0],
        [0, 0, 1, l1],
        [0, 0, 0, 1]
    ])
    A2 = np.array([
        [np.cos(q2), -np.sin(q2), 0, 0],
        [0, 0, -1, 0],
        [np.sin(q2), np.cos(q2), 0, 0],
        [0, 0, 0, 1]
    ])
    A3 = np.array([
        [np.cos(q3), -np.sin(q3), 0, 0],
        [0, 0, 1, l2],
        [-np.sin(q3), -np.cos(q3), 0, 0],
        [0, 0, 0, 1]
    ])
    A4 = np.array([
        [1, 0, 0, l3],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    T = np.linalg.multi_dot([A1, A2, A3, A4])
    P = T[:3, 3]

    return P

# PUMA RRR Inverse Kinematics
def puma_inv(x, y, z):
    q1 = np.arctan2(y, x)

    r = np.sqrt(x**2 + y**2) #distance from the origin to the wrist
    D = (r**2 + (z - l1)**2 - l2**2 - l3**2) / (2 * l2 * l3)
    q2 = np.arctan2(np.sqrt(1 - D**2), D)

    gamma = np.arctan2(z - l1, r)
    beta = np.arctan2(l3 * np.sin(q2), l2 + l3 * np.cos(q2))
    q3 = np.pi/2 - (gamma - beta)

    return q1,q2,q3 #in radians

def check_ws(P):
    x,y,z=P
    q1,q2,q3=puma_inv(x,y,z)
    x1,y1,z1=puma_endeff(q1,q2,q3)
    p=np.array([x,y,z])
    p1=np.array([x1,y1,z1])

    if (np.linalg.norm(p)-np.linalg.norm(p1))<0.005:
        return True
    else:
        return False

#max corner points
A = (0.40, 0.06, 0.1)
B = (0.40, 0.01, 0.1)
C = (0.35, 0.01, 0.1)
D =(0.35, 0.06, 0.1)
P=[A,B,C,D]
k=0 #variable to confirm that all corners are in the ws

#check if all corners in the ws
for i in P:
    t=check_ws(i)
    if t==False:
        print("point",i,"is not in workspace")
        k=0
        break
    k=1

if k==1:
    print('all corners are in the ws of the robot')