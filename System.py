'''Imports'''
import matplotlib.pyplot as plt
import numpy as np
import math
import random
import matplotlib.animation as animation
'''
Class defintion:
Robot class contains the continuous and discrete states of each robot.
z= [x-position, y-position, theta (direction)]
tau = tagging ability
-smaller or equal to 0; has tagging ability
-else; no tagging ability
q=activation state
-0=active; 1=deactivated
eta=carrying flag
-0=not carrying flag; 1=carrying flag
ti is an array that saves the value of these parameters for every time step
'''
class Robot():
    '''
    Initializes the robots with the starting values. 
    '''
    def __init__(self, z, tau, q, eta):
        self.z = z 
        self.tau = tau 
        self.q = q
        self.eta = eta
        self.ti=[]
        self.add_time_step(z,tau,q,eta)
        self.time=0

    '''
    Adds the current time step to the ti array.
    '''
    def add_time_step(self,z,tau,q,eta):
        self.ti.append([z,tau,q,eta])

    def getZ(self): 
        return self.z
    
    def getTau(self): 
        return self.tau
    
    def getQ(self): 
        return self.q
    
    def getEta(self): 
        return self.eta
    
    def getTime(self, time):
        return self.ti[time]
    
    def getData(self):
        return self.ti
    '''
    Updates the current variables with the ones given from the system.
    '''
    def step (self, z, tau, q, eta, time):
        z[0]=round(z[0],3)
        z[1]=round(z[1],3)
        self.add_time_step(z,tau,q,eta)
        self.z=z
        self.tau=tau
        self.q=q
        self.eta=eta

'''
Sets the bounds of the map
'''
x_f = np.array([-80, 80, 80, -80, -80])
y_f = np.array([-40, -40, 40, 40, -40])
X = np.array([-80, -40, 80, 40])  
X_B = np.array([-80, -40, 0, 40])  
X_R = np.array([0, -40, 80, 40])  


l = np.linspace(0, 2 * np.pi, 200)  
flx = np.sin(l)
fly = np.cos(l)
'''
Define radius of flag capture area and robot tagging area.
'''
gf = 10  
gc = 10 
'''
Define flag position.
'''
FB = np.array([-60, 0])  
FR = np.array([60, 0])   
'''
Define number of Robots
'''
b = 3 
r = 3  
'''
Set inital distance from flag
'''
ip = 25  
'''
Define time out time
'''
barT = 2  

'''
Dictionaries to store the robots in each team. 
'''
RobotsB={}
RobotsR={}
'''
Returns true if point is in rectangle; else false.
'''
def pointInRectangle(p,rect):
    x=p[0]
    y=p[1]
    return (rect[0]<=x<=rect[1]) and (rect[2]<=y<=rect[3])
'''
Returns true if point is in rectangle; else false.
'''
def pointInCircle(p,center,radius):
    x=p[0]
    y=p[1]
    distance = (((x-center[0])**2) + ((y-center[1])**2))**0.5
    return (distance<=radius)
'''
Returns the new time for tagging ability. 
'''
def Tagging_Ability(tau,barT):
    return barT
'''
Calculates the angle between two points
'''
def calculate_angle(x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    angle = math.atan2(dy, dx)
    return angle

'''
Controller
'''
def Angle_Control(ti, MRobots, ORobots, z, tau, q, eta, MF, OF, NMA, NOA, dt):

    if q == 1: #or (MRobots.getEta() == 1 and MRobots.getZ()[0] * MF[0] >= 0):
        theta = calculate_angle(MRobots.getZ()[0], MRobots.getZ()[1], MF[0], MF[1])
    else:
        theta = np.sin(ti*dt*4)
    return theta
    
'''
Robot dynamics given current state, velocity, and theta
'''
def robot_dynamics(z, u, dt):
    
    u1 = 0.5*u[0]
    u2 = u[1]

    p1 = z[0]
    p2 = z[1]
    theta = z[2]

    p1_plus = p1 + u1 * np.cos(u2) * dt
    p2_plus = p2 + u1 * np.sin(u2) * dt
    theta_plus = u2 

    z_plus = [p1_plus, p2_plus, theta_plus]
    
    return z_plus

'''
Initializes blue robots
'''
for k in range(1, b+1):
    z = np.array([FB[0] + ip * np.cos(-np.pi / 2 + k * np.pi / (b + 1)),
                FB[1] + ip * np.sin(-np.pi / 2 + k * np.pi / (b + 1)),
                -np.pi / 2 + np.pi * np.random.rand()])
    tau = 0
    q = 0
    eta = 1
    RobotsB[f"Robot{k}B"] = Robot(z,tau,q,eta)

RobotsB['mu']=1

'''
Initializes red robots
'''
for k in range(1, r+1):
    z = np.array([FR[0] + ip * np.cos(np.pi / 2 + k * np.pi / (r + 1)),  
                FR[1] + ip * np.sin(np.pi / 2 + k * np.pi / (r + 1)),  
                np.pi / 2 + np.pi * np.random.rand()])
    tau = 0
    q = 0
    eta = 1
    RobotsR[f"Robot{k}R"] = Robot(z,tau,q,eta)

RobotsR['mu']=1

'''
Set time span
'''
TSPAN=[0,10]
dt=0.01

'''
Main Simulation Loop
'''
for ti in range(int(TSPAN[1]/dt)):
    for k in range (1,b+1):
        for i in range(1,r+1):
            CurRobotB=RobotsB[f"Robot{k}B"]
            CurRobotR=RobotsR[f"Robot{i}R"]
            zB=CurRobotB.getZ()
            tauB=CurRobotB.getTau()
            qB=CurRobotB.getQ()
            etaB=CurRobotB.getEta()
            muB=RobotsB['mu']

            zR=CurRobotR.getZ()
            tauR=CurRobotR.getTau()
            qR=CurRobotR.getQ()
            etaR=CurRobotR.getEta()
            muR=RobotsR['mu']
            
            if (pointInRectangle(zB[0:2],X_B) and qB==0 and tauB<=0 and 
                pointInRectangle(zR[0:2], X_B) and qR()==0 and pointInCircle(zR[0:2],zB()[0:2],gc)):
                tauB=Tagging_Ability(tauB,barT)
                qR=1-qR
                if etaR==1 and muB==0:
                    muB=1-muB
                    etaR=1-etaR
            
            if (pointInRectangle(zB, X_R) and qB == 0 and
                pointInRectangle(zR, X_R) and qR == 0 and tauR <= 0 and
                pointInCircle(zB, zR, gc)) or (
                    not pointInRectangle(zB, X) and qB == 0) or (
                        pointInCircle(zB, FB, gf) and qB == 1): 

                if (pointInRectangle(zB, X_R) and qB == 0 and
                    pointInRectangle(zR, X_R) and qR == 0 and tauR <= 0 and
                    pointInCircle(zB, zR, gc)):

                    tauR = Tagging_Ability(tauR, barT)

                if not (pointInCircle(zB, FB, gf) and qB == 1) and etaB == 1 and muR == 0:
                    etaB = 1 - etaB
                    muR = 1 - muR

                qB = 1 - qB
            if (pointInCircle(zB, FR, gf) and qB == 0 and etaB == 0 and muR == 1) or \
                (pointInCircle(zB, FB, gf) and qB == 0 and etaB == 1 and muR == 0):  

                muR  = 1 - muR 
                etaB = 1 - etaB  

            if (pointInCircle(zR, FB, gf) and qR == 0 and etaR == 0 and muB == 1) or \
               (pointInCircle(zR, FR, gf) and qR == 0 and etaR == 1 and muB == 0) or \
               (not pointInRectangle(zR, X) and qR == 0 and etaR == 1 and muB == 0):  

                if not pointInRectangle(zR, X):
                    qR = 1 - qR  

                etaR  = 1 - etaR  
                muB = 1 - muB 
        theta = Angle_Control(ti,CurRobotB,CurRobotR,zB,tauB,qB,etaB,FB,FR,b,r, dt)
        zB = robot_dynamics(zB, [50, theta],dt)
        CurRobotB.step(zB,tauB,qB,etaB,ti)
    RobotsB['mu']=muB
    RobotsR['mu']=muR
    for i in range (1,r+1):
        for k in range(1,b+1):
            CurRobotB=RobotsB[f"Robot{k}B"]
            CurRobotR=RobotsR[f"Robot{i}R"]
            zB=CurRobotB.getZ()
            tauB=CurRobotB.getTau()
            qB=CurRobotB.getQ()
            etaB=CurRobotB.getEta()
            muB=RobotsB['mu']

            zR=CurRobotR.getZ()
            tauR=CurRobotR.getTau()
            qR=CurRobotR.getQ()
            etaR=CurRobotR.getEta()
            muR=RobotsR['mu']

            if (pointInRectangle(zR, X_B) and qR == 0 and
                pointInRectangle(zB, X_B) and qB == 0 and tauB <= 0 and
                pointInCircle(zR, zB, gc)) or \
               (not pointInRectangle(zR, X) and qR == 0) or \
               (pointInCircle(zR, FR, gf) and qR == 1):  

                if (pointInRectangle(zR, X_B) and qR == 0 and
                    pointInRectangle(zB, X_B) and qk == 0 and tauB <= 0 and
                    pointInCircle(zR, zB, gc)):

                    tauB = Tagging_Ability(tauB, barT)

                if not (pointInCircle(zR, FR, gf) and qR == 1) and etaR == 1 and muB == 0:
                    etaR = 1 - etaR  
                    muB = 1 - muB
                
                qR = 1 - qR
            
            if (pointInCircle(zR, FB, gf) and qR == 0 and etaR == 0 and muB == 1) or \
               (pointInCircle(zR, FR, gf) and qR == 0 and etaR == 1 and muB == 0):  
                muB = 1 - muB  
                etaR = 1 - etaR  

            if (pointInCircle(zB, FR, gf) and qB == 0 and etaB == 0 and muR == 1) or \
               (pointInCircle(zB, FB, gf) and qB == 0 and etaB == 1 and muR == 0) or \
               (not pointInRectangle(zB, X) and qB == 0 and etaB == 1 and muR == 0): 
                if not pointInRectangle(zB, X):  
                    qk = 1 - qk  
                    etaB = 1 - etaB  
                muR = 1 - muR 
        theta = Angle_Control(ti,CurRobotR,CurRobotB,zR,tauR,qR,etaR,FR,FB,b,r, dt)
        zR = robot_dynamics(zR, [50, theta],dt)
        CurRobotR.step(zR,tauR,qR,etaR,ti)
    RobotsB['mu']=muB
    RobotsR['mu']=muR


#print(RobotsR["Robot1R"].getData())

#for k in range(1, b+1):
    #print(RobotsB[f"Robot{k}B"].getData())


#for k in range(1, r+1):
    #print(RobotsR[f"Robot{k}R"].getData())






fig, ax = plt.subplots(figsize=(6, 6))

ax.set_xlim(x_f[0], x_f[2])
ax.set_ylim(y_f[0], y_f[2])
ax.set_aspect('equal')
ax.set_xlabel("X")
ax.set_ylabel("Y")
circleB = plt.Circle((FB[0], FB[1]), gf, color='b', fill=False)
ax.add_patch(circleB)
circleR = plt.Circle((FR[0], FR[1]), gf, color='r', fill=False)
ax.add_patch(circleR)

robot_scatter_1B = ax.scatter(RobotsB["Robot1B"].getTime(0)[0][0], RobotsB["Robot1B"].getTime(0)[0][1], c="b", s=5)
robot_scatter_1R = ax.scatter(RobotsR["Robot1R"].getTime(0)[0][0], RobotsR["Robot1R"].getTime(0)[0][1], c="r", s=5)
robot_scatter_2B = ax.scatter(RobotsB["Robot2B"].getTime(0)[0][0], RobotsB["Robot2B"].getTime(0)[0][1], c="b", s=5)
robot_scatter_2R = ax.scatter(RobotsR["Robot2R"].getTime(0)[0][0], RobotsR["Robot2R"].getTime(0)[0][1], c="r", s=5)
robot_scatter_3B = ax.scatter(RobotsB["Robot3B"].getTime(0)[0][0], RobotsB["Robot3B"].getTime(0)[0][1], c="b", s=5)
robot_scatter_3R = ax.scatter(RobotsR["Robot3R"].getTime(0)[0][0], RobotsR["Robot3R"].getTime(0)[0][1], c="r", s=5)
flag_scatter_B = ax.scatter(FB[0], FB[1], c="b", s=5)
flag_scatter_R = ax.scatter(FR[0], FR[1], c="r", s=5)
ax.set(xlim=[x_f[0], x_f[2]], ylim=[y_f[0], y_f[2]])


def update(i):
    x = RobotsB["Robot1B"].getTime(i)[0][0]
    y = RobotsB["Robot1B"].getTime(i)[0][1]
    data = np.stack([x, y]).T
    robot_scatter_1B.set_offsets(data)
    x = RobotsR["Robot1R"].getTime(i)[0][0]
    y = RobotsR["Robot1R"].getTime(i)[0][1]
    data = np.stack([x, y]).T
    robot_scatter_1R.set_offsets(data)
    x = RobotsB["Robot2B"].getTime(i)[0][0]
    y = RobotsB["Robot2B"].getTime(i)[0][1]
    data = np.stack([x, y]).T
    robot_scatter_2B.set_offsets(data)
    x = RobotsR["Robot2R"].getTime(i)[0][0]
    y = RobotsR["Robot2R"].getTime(i)[0][1]
    data = np.stack([x, y]).T
    robot_scatter_2R.set_offsets(data)
    x = RobotsB["Robot3B"].getTime(i)[0][0]
    y = RobotsB["Robot3B"].getTime(i)[0][1]
    data = np.stack([x, y]).T
    robot_scatter_3B.set_offsets(data)
    x = RobotsR["Robot3R"].getTime(i)[0][0]
    y = RobotsR["Robot3R"].getTime(i)[0][1]
    data = np.stack([x, y]).T
    robot_scatter_3R.set_offsets(data)
    return robot_scatter_1B, robot_scatter_1R,robot_scatter_2B, robot_scatter_2R,robot_scatter_3B, robot_scatter_3R,flag_scatter_B,flag_scatter_R


anim = animation.FuncAnimation(fig,update,frames=200, interval=10)#(int(TSPAN[1]/dt)))

plt.show()

