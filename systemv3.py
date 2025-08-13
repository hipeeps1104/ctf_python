'''Imports'''
import matplotlib.pyplot as plt
import numpy as np
import math
import random
import matplotlib.animation as animation
import time
import matplotlib.patches as patches
from sklearn.neighbors import NearestNeighbors
import matplotlib.transforms as transforms


# Robot class contains the continuous and discrete states of each robot.

class Robot():
    
    # Initializes the robots with the starting values.
    
    def __init__(self, z, tau, q, eta):
        self.z = z         # [x-position, y-position, thetab (direction)]
        self.tau = tau     # tagging ability: -smaller or equal to 0; has tagging ability; else; no tagging ability
        self.q = q         # activation state: 0=active; 1=deactivated
        self.eta = eta     # carrying flag: 0=not carrying flag; 1=carrying flag
        self.datastorage=[]         # an array that saves the value of these parameters for every time step
        self.add_time_step(z,tau,q,eta)
        self.time=0

    # Adds the current time step to the ti array.

    def add_time_step(self,z,tau,q,eta):
        self.datastorage.append([z,tau,q,eta])

    def getZ(self): 
        return self.z
    
    def getTau(self): 
        return self.tau
    
    def getQ(self): 
        return self.q
    
    def getEta(self): 
        return self.eta
    
    def getTime(self, time):
        return self.datastorage[time]
    
    def getData(self):
        return self.datastorage
 
    # Updates the current variables with the ones given from the system.
    def step (self, z, tau, q, eta, time):
        z[0]=round(z[0],3)
        z[1]=round(z[1],3)
        self.add_time_step(z,tau,q,eta)
        self.z=z
        self.tau=tau
        self.q=q
        self.eta=eta

    def upstate (self, z, tau, q, eta, time):
        self.tau=tau
        self.q=q
        self.eta=eta



# Sets the bounds of the map, Dimension of the Playing Field
x_f = np.array([-80, 80, 80, -80, -80])
y_f = np.array([-40, -40, 40, 40, -40])
X = np.array([-80, -40, 80, 40])  # Playing Field
X_B = np.array([-80, -40, 0, 40]) # Team Blue Region 
X_R = np.array([0, -40, 80, 40])  # Team Read Region


#Define radius of flag capture area and robot tagging area.
gf = 10  # Radius of Flag Capture Region
gc = 10  # Radius of Robot Tagging Region

#Define flag position.
FB = np.array([-60, 0])  # Flag Postion Team B
FR = np.array([60, 0])   # Flag Postion Team R

#Define number of Robots
b = 3 # Number of robots in Team B
r = 3  # Number of robots in Team R

#Set inital distance from flag
ip = 25  # Initial distance of robots from the flag

# Define time out time
barT = 2  # Timeout after being tagged

# Dictionaries to store the robots in each team. 
RobotsB={}
RobotsR={}

# Returns true if point is in rectangle; else false.
def pointInRectangle(p,rect):
    x=p[0]
    y=p[1]
    return (rect[0]<=x<=rect[2]) and (rect[1]<=y<=rect[3])

# Returns true if point is in Circle; else false.
def pointInCircle(p,center,radius):
    x=p[0]
    y=p[1]
    distance = (((x-center[0])**2) + ((y-center[1])**2))**0.5
    return (distance<=radius)

#Calculates distance between two points
def calcDis(p1,p2):
    x1=p1[0]
    y1=p1[1]
    x2=p2[0]
    y2=p2[1]
    distance = (((x1-x2)**2) + ((y1-y2)**2))**0.5
    return distance

# Returns the new time for tagging ability. 
def Tagging_Ability(tau,barT):
    return barT

# Calculates the angle between two points
def calculate_angle(x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    angle = math.atan2(dy, dx)
    return angle

#Draw a rectangle whose hinge (pivot) is the midpoint of a side.
def rect_with_side_hinge(ax, hinge_xy, angle, width, height, side='bottom', radians=False, **patch_kwargs):
    hx, hy = hinge_xy

    # lower-left corner relative to the hinge, depending on which side is the hinge
    if side == 'bottom':
        x0, y0 = -width/2, 0
    elif side == 'top':
        x0, y0 = -width/2, -height
    elif side == 'left':
        x0, y0 = 0, -height/2
    elif side == 'right':
        x0, y0 = -width, -height/2
    else:
        raise ValueError("side must be 'bottom', 'top', 'left', or 'right'")

    theta = angle if radians else np.deg2rad(angle)

    # rotate around the hinge (0,0 in local coords), then translate to (hx, hy)
    base = transforms.Affine2D().rotate(theta).translate(hx, hy)

    rect = patches.Rectangle((x0, y0), width, height,
                             transform=base + ax.transData,
                             **patch_kwargs)
    ax.add_patch(rect)
    return rect

#Updates rectangle with new angle and point
def update_rect_transform(ax, rect, hinge_xy, angle, side='bottom', radians=False):
    import matplotlib.transforms as transforms
    hx, hy = hinge_xy
    theta = angle if radians else np.deg2rad(angle)
    base = transforms.Affine2D().rotate(theta).translate(hx, hy)
    rect.set_transform(base + ax.transData)

'''
Controller
'''
def Angle_Control1 (ti, MRobots, ORobots, z, tau, q, eta, Mmu, Omu, MF, OF, NMA, NOA, MRobs, ORobs, color, robonumber, dt):
    # Get predicted positions of opponents and attackers at current time step
    [Opponent_Positions_0, Attackers_Positions_0] = Positions_in_Matrix(MRobots.getZ()[0], ORobs, NOA, ti)

    # Filter only active opponents/attackers that are in my robot's current half
    [Opponent_Positions, active_opponents] = multiplyFirstElement(MRobots.getZ()[0], Opponent_Positions_0)
    [Attackers_Positions, active_attackers] = multiplyFirstElement(MRobots.getZ()[0], Attackers_Positions_0)

    # Determine the closest opponent and attacker to my robot
    Closest_Opponent_index = closest_opponent(MRobots.getZ()[0:2], Opponent_Positions)
    Closest_Attacker_index = closest_opponent(MRobots.getZ()[0:2], Attackers_Positions)

    theta = 0  # Default angle initialization

    # Case 1: Robot is deactivated (Q==1) OR carries the flag AND is still on my side (eta==1 and same sign with MF)
    if MRobots.getQ() == 1 or (MRobots.getEta() == 1 and MRobots.getZ()[0] * MF[0] >= 0):
        # Set theta to head toward my flag (return to base)
        theta = calculate_angle(MRobots.getZ()[0], MRobots.getZ()[1], MF[0], MF[1])
        return theta
    
    # Case 2: Robot carries the flag AND is on opponent's side (eta==1 and position < 0)
    elif MRobots.getEta() == 1 and MRobots.getZ()[0]*MF[0] <= 0:
        if active_opponents>0:
            # Return flag while avoiding closest opponent 
            m_x=-(MRobots.getZ()[1]-ORobots.getZ()[1])/(MRobots.getZ()[0]-ORobots.getZ()[0])
            diff1 = MRobots.getZ()[0:1] - OF
            diff2 = ORobots.getZ()[0:1] - OF
            n_x=0.5 * (np.linalg.norm(diff1)**2 - np.linalg.norm(diff2)**2) / (MRobots.getZ()[0] - ORobots.getZ()[0])
            yst = 40 * np.sign(m_x)
            numer = m_x * yst + n_x - (MRobots.getZ()[0] - OF[0])
            denom = np.linalg.norm(np.array([m_x * yst + n_x, yst]) - diff1)
            theta = np.sign(m_x) * np.arccos(numer / denom)
            return theta
        else:
            # No opponents around, return in straight line
            theta = (1 + np.sign(MRobots.getZ()[0])) * np.pi / 2
            return theta

    # Case 3: Robot is active, on my side, not carrying the flag, and there is an active threat (attackers or opponents)
    elif active_attackers+active_opponents>0 and MRobots.getEta()==0 and MRobots.getQ()==0 and MRobots.getTau() <=0 and MRobots.getZ()[0]*MF[0]>0:
        # If has flag and attackers present
        if Mmu == 1 and active_attackers > 0:
            # Set theta to defend the flag
            vec1 = ORobots.getZ()[0:1] - MF
            vec2 = MRobots.getZ()[0:1] - MF
            vec_diff = np.asarray(ORobots.getZ()[0:1]) - np.asarray(MRobots.getZ()[0:1])
            denom = np.linalg.norm(vec_diff)**2

            xst = 0.5 * (np.linalg.norm(vec1)**2 - np.linalg.norm(vec2)**2) * \
            (ORobots.getZ()[0] - MRobots.getZ()[0]) / denom

            yst = 0.5 * (np.linalg.norm(vec1)**2 - np.linalg.norm(vec2)**2) * \
            (ORobots.getZ()[1] - MRobots.getZ()[1]) / denom

            diff_vec = np.array([xst, yst]) - MRobots.getZ()[0:1]
            theta = np.arccos((xst - MRobots.getZ()[0]) / np.linalg.norm(diff_vec))
            return theta
        # If flag is gone and opponents present
        elif Mmu == 0 and active_opponents > 0:
            # Set theta to protect boundary
            m_x = -(ORobots.getZ()[1] - MRobots.getZ()[1]) / (ORobots.getZ()[0] - MRobots.getZ()[0])

            vec1 = ORobots.getZ()[0:1] - OF
            vec2 = MRobots.getZ()[0:1] - OF
            n_x = 0.5 * (np.linalg.norm(vec1)**2 - np.linalg.norm(vec2)**2) / \
            (ORobots.getZ()[0] - MRobots.getZ()[0])

            yst = 40 * np.sign(m_x)

            numer = m_x * yst + n_x - (MRobots.getZ()[0] - OF[0])
            denom = np.linalg.norm(np.array([m_x * yst + n_x, yst]) - vec2)

    
            theta = (MF[0] < 0) * np.pi + np.sign(MF[0]) * np.arccos(numer / denom)
            return theta
    elif robonumber == 2 and color == 'b' and Omu==1:
        if MRobots.getZ()[0]*MF[0]>0:
            theta=(1+np.sign(MRobots.getZ()[0]))*np.pi/2
            return theta
        else:
            theta=calculate_angle(MRobots.getZ()[0],MRobots.getZ()[1],OF[0],OF[1])
            return theta
    # General fallback behavior
    elif color == 'r':
        theta=np.sin(ti*dt*4)+np.pi
        return theta
    else:
        theta=np.sin(ti*dt*4)
        return theta
    return theta


'''
MRpx: x position of my robot
ORobots: Opponent team's vector
NOA: Number of robots in opponent team
'''
def Positions_in_Matrix(MRpx, ORobots, NOA,ti):
    # Initialize empty arrays for storing positions of boundary defenders and attackers
    Boundary_defenders_pos=np.empty((0, 2)) 
    attackers_pos=np.empty((0, 2)) 
    # Iterate over all opponent robots
    for m in range(NOA):
        # Get opponent robot state at time ti
        z = ORobots[f"Robot{m+1}"].getZ()
        tau = ORobots[f"Robot{m+1}"].getTau()
        q = ORobots[f"Robot{m+1}"].getQ()

        # Compute distance magnitudes (for comparing closeness to boundary center)
        norm_mrpx = np.linalg.norm(MRpx)
        norm_z = np.linalg.norm(z[0])
        # Default modifier (no change)
        modifier_vec = np.array([1.0, 1.0])
        # Check if both robots are on the same half
        if (MRpx * z[0]) > 0:
            # If my robot is farther from center than the opponent robot
            if norm_mrpx > norm_z:
                # If opponent has tagging ability or is activated =  boundary defender
                if (tau > 0) or q:
                    modifier_vec = np.array([-100.0, 0.0])
            # Else if my robot is closer than opponent = boundary defender
            elif norm_mrpx < norm_z:
                modifier_vec = np.array([-100.0, 0.0])
        # Apply the modifier to opponent's position
        new_position = modifier_vec * z[0:1]
        # Add to boundary defenders position matrix
        Boundary_defenders_pos = np.vstack([Boundary_defenders_pos, new_position])

        # Reset modifier
        modifier_vec = np.array([1.0, 1.0])

        # Determine conditions for attacker classification
        same_direction = MRpx * z[0] > 0  # On same side
        closer_than_z = norm_mrpx < norm_z # closer to the center
        special_condition = (tau > 0) or q # Opponent can tag or is activated

        # If all attacker conditions are met = active attacker
        if same_direction and closer_than_z and special_condition:
            modifier_vec = np.array([-100.0, 0.0])
        
        # Apply modifier to position
        new_position = modifier_vec * z[0:1]
        # Add to attackers position matrix
        attackers_pos = np.vstack([attackers_pos, new_position])
    # Return position arrays for defenders and attackers
    return Boundary_defenders_pos, attackers_pos

# Multiply the x-values of robots that are on the opposite side by 100
def multiplyFirstElement(MRpx, OPosMat):
    # Determine the sign of my robot's x-position to identify my field half
    signNum=0
    if MRpx>0:
        signNum=1 # Right side of field
    elif MRpx<0:
        signNum=-1 # Left side of field

    # Initialize an empty array to store binary flags: 1 = robot is in opponent half, 0 = same half
    ORobotsInOpHalf = np.empty((0,1))

    # For each opponent position, check if it's in the opponent half 
    for i in range(0, OPosMat.shape[0]):
        if OPosMat[i][0] * signNum <0:
            ORobotsInOpHalf=np.vstack([ORobotsInOpHalf, 1]) # Opponent half
        else:
            ORobotsInOpHalf=np.vstack([ORobotsInOpHalf, 0]) # My half or on boundary
    # Initialize result matrix for storing transformed positions
    resultMat=[]
    # Iterate over opponent robots starting from index 1 
    for i in range(0, ORobotsInOpHalf.shape[0]):
        if ORobotsInOpHalf[i]==1:
            # Emphasize x-distance (multiply x by 100) to exaggerate horizontal separation
            modified_row = [OPosMat[i][0] * 100, OPosMat[i][1]]
            resultMat.append(np.asarray(modified_row))
        else:
            resultMat.append(np.asarray(OPosMat[i]))
     # Convert result list to numpy array
    # Calculate number of same-half robots: total - number in opponent half
    NSH = len(OPosMat)-sum(ORobotsInOpHalf)
    # Return filtered and transformed positions, and count of same-half robots
    return resultMat,NSH

# Finds the closest opponent via a kd-tree
def closest_opponent(my_position, opponents_positions):
    # Combine my robot's position with the opponent positions into a single array.
    # This allows using a k-d tree to efficiently find the nearest neighbor.
    all_positions = np.vstack([my_position, opponents_positions])
    # Create a KD-tree from all positions (including my own) for fast nearest-neighbor search
    kdtree = NearestNeighbors(n_neighbors=2, algorithm='kd_tree').fit(all_positions)
    # Find the two nearest neighbors to my position.
    # The first will be myself (distance = 0), and the second will be the actual closest opponent.
    distances, indices = kdtree.kneighbors([my_position], n_neighbors=2)
    # Extract index of the closest opponent (second neighbor, because first is self)
    closest_particle_index = indices[0][1]
    # Extract the distance to the closest opponent
    closest_particle_distance = distances[0][1]
    # Return both the index (in the combined array) and distance of the closest opponent
    return closest_particle_index, closest_particle_distance

'''
Robot dynamics given current state, velocity, and thetab
'''
def robot_dynamics(z, u, dt):
    
    u1 = u[0]
    u2 = u[1]

    p1 = z[0]
    p2 = z[1]
    thetab = z[2]

    p1_plus = p1 + u1 * np.cos(u2) * dt
    p2_plus = p2 + u1 * np.sin(u2) * dt
    thetab_plus = u2 

    z_plus = [p1_plus, p2_plus, thetab_plus]
    
    return z_plus

'''
Initializes blue robots
'''
for k in range(1, b+1):
    z = np.array([FB[0] + ip * np.cos(-np.pi / 2 + k * np.pi / (b + 1)),
               FB[1] + ip * np.sin(-np.pi / 2 + k * np.pi / (b + 1)),
               -np.pi / 2 + np.pi * np.random.rand()])
    # z = np.array([FB[0] + ip * np.cos(-np.pi / 2 + k * np.pi / (3)),
    #             FB[1] + ip * np.sin(-np.pi / 2 + k * np.pi / (3)),
    #             -np.pi / 2 + np.pi * np.random.rand()])
    tau = 0 # Starts with tagging ability
    q = 0   # Start with active
    eta = 0 # Starts not carrying the flag
    RobotsB[f"Robot{k}"] = Robot(z,tau,q,eta)

RobotsB['mu']=[1] # Flag in place

'''
Initializes red robots
'''
for k in range(1, r+1):
    z = np.array([FR[0] + ip * np.cos(np.pi / 2 + k * np.pi / (r + 1)),  
                FR[1] + ip * np.sin(np.pi / 2 + k * np.pi / (r + 1)),  
                np.pi / 2 + np.pi * np.random.rand()])
    tau = 0 # Starts with tagging ability
    q = 0   # Starts not tagged
    eta = 0 # Starts not carrying the flag
    RobotsR[f"Robot{k}"] = Robot(z,tau,q,eta)

RobotsR['mu']=[1] #Flag in place

'''
Set time span
'''
# Simulation Horizon
#TSPAN=[0,500] # Second entry is the maximum amount of seconds allowed
TSPAN=[0,10]
dt=0.01
speed=50

'''
Main Simulation Loop
'''
# System Evolution

for ti in range(1,int(TSPAN[1]/dt)+1):

    ## Team Blue
    for k in range (1,b+1):
        CurRobotB=RobotsB[f"Robot{k}"]
        zB=CurRobotB.getZ()
        tauB=CurRobotB.getTau()
        qB=CurRobotB.getQ()
        etaB=CurRobotB.getEta()
        muB=RobotsB['mu'][-1]
        # Jump of logic variables:
        for i in range(1,r+1):

            CurRobotR=RobotsR[f"Robot{i}"]
            zR=CurRobotR.getZ()
            tauR=CurRobotR.getTau()
            qR=CurRobotR.getQ()
            etaR=CurRobotR.getEta()
            muR=RobotsR['mu'][-1]

             # Tagging in blue zone (Blue tag Red)
            if (pointInRectangle(zB[0:2],X_B) and qB==0 and tauB<=0 and 
                pointInRectangle(zR[0:2], X_B) and qR==0 and pointInCircle(zR[0:2],zB[0:2],gc)):

                tauB=Tagging_Ability(tauB,barT) # Jump in tagging ability 
                qR = 1-qR # Jump in tagged state 
                # Jump if carrying flag
                if etaR==1 and muB==0:
                    muB = 1-muB # Blue base get flag
                    etaR = 1-etaR # Red robot lose flag
            
            # Tagged in red zone
            if (pointInRectangle(zB[0:2], X_R) and qB == 0 and
                pointInRectangle(zR[0:2], X_R) and qR == 0 and tauR <= 0 and pointInCircle(zB[0:2], zR[0:2], gc)) \
                or (not(pointInRectangle(zB[0:2], X)) and qB == 0) \
                or (pointInCircle(zB[0:2], FB, gf) and qB == 1): 
                # Tagged in red zone
                if (pointInRectangle(zB[0:2], X_R) and qB == 0 and
                    pointInRectangle(zR[0:2], X_R) and qR == 0 and tauR <= 0 and
                    pointInCircle(zB[0:2], zR[0:2], gc)):

                    tauR = Tagging_Ability(tauR, barT) # Jump in tagging ability 
                
                # Either robot B gets tagged or leaves X while carrying the flag
                if  not(pointInCircle(zB[0:2], FB, gf) and qB == 1) and etaB == 1 and muR==0: # and muR == 0: # Carries the flag
                    etaB = 1 - etaB # Not carrying flag anymore
                    muR = 1 - muR # Red Flag returned to base

                qB = 1 - qB

            #  First case accounted above: tagged in red zone w flag
            if (pointInCircle(zB[0:2], FR[0:2], gf) and qB == 0 and etaB == 0 and muR ==1 ) \
                or (pointInCircle(zB[0:2], FB[0:2], gf) and qB == 0 and etaB == 1 and muR==0):

                muR  = 1 - muR # Jump in red flag position state (captured or dropped)
                etaB = 1 - etaB  # Jump in carrying the flag state
            
            # First case accounted above: tagged in blue zone w flag
            if (pointInCircle(zR[0:2], FB[0:2], gf) and qR == 0 and etaR == 0 and muB==1) \
                or (pointInCircle(zR[0:2], FR[0:2], gf) and qR == 0 and etaR == 1 and muB==0) \
                or (not(pointInRectangle(zR[0:2], X)) and qR == 0 and etaR == 1 and muB==0):  

                if not(pointInRectangle(zR[0:2], X)):
                    qR = 1 - qR  
                etaR  = 1 - etaR  # Jump in carrying the flag state
                muB = 1 - muB  # Jump in blue flag position state (captured or dropped)
            
            
            RobotsR['mu'][-1]=muR
            RobotsB['mu'][-1]=muB
            CurRobotR.upstate(zR,tauR-dt,qR,etaR,ti)
   
        #  Controller 
        thetab = Angle_Control1(ti,CurRobotB,CurRobotR,zB,tauB,qB,etaB,muB,muR,FB,FR,b,r, RobotsB,RobotsR, 'b', k, dt)
        # Flow of robot
        zB = robot_dynamics(zB, [speed, thetab],dt)
        # flow of logic variable
        #CurRobotB.step(zB,tauB,qB,etaB,ti)
        CurRobotB.step(zB,tauB-dt,qB,etaB,ti)
        ## Flow of logic variables: where
        RobotsB['mu'][-1]=muB
        RobotsR['mu'][-1]=muR
    RobotsR['mu'][-1]=muR   
    RobotsB['mu'].append(muB)

    # Team Red
    for i in range (1,r+1):
        CurRobotR=RobotsR[f"Robot{i}"]
        zR=CurRobotR.getZ()
        tauR=CurRobotR.getTau()
        qR=CurRobotR.getQ()
        etaR=CurRobotR.getEta()
        muR=RobotsR['mu'][-1]
        for k in range(1,b+1):
            CurRobotB=RobotsB[f"Robot{k}"]
            zB=CurRobotB.getZ()
            tauB=CurRobotB.getTau()
            qB=CurRobotB.getQ()
            etaB=CurRobotB.getEta()
            muB=RobotsB['mu'][-1]

            ###### Tagged in blue zone
            if (pointInRectangle(zR[0:2], X_B) and qR == 0 and
                pointInRectangle(zB[0:2], X_B) and qB == 0 and tauB <= 0 and pointInCircle(zR[0:2], zB[0:2], gc)) \
                or (not pointInRectangle(zR[0:2], X) and qR == 0) \
                or (pointInCircle(zR[0:2], FR[0:2], gf) and qR == 1): 
            # tagged in blue zone
                if (pointInRectangle(zR[0:2], X_B) and qR == 0 and
                    pointInRectangle(zB[0:2], X_B) and qB == 0 and tauB <= 0 and
                    pointInCircle(zR[0:2], zB[0:2], gc)):

                    tauB = Tagging_Ability(tauB, barT)

                # Either tB gets tagged or leaves X while carrying the flag 
                if  not(pointInCircle(zR[0:2], FR[0:2], gf) and qR == 1) \
                    and etaR == 1 and muB == 0:
                    etaR = 1 - etaR  
                    muB = 1 - muB
                qR = 1 - qR   
            
            ## First case accounted above: tagged in blue zone w flag
            if (pointInCircle(zR[0:2], FB[0:2], gf) and qR == 0 and etaR == 0 and muB ==1) or \
            (pointInCircle(zR[0:2], FR[0:2], gf) and qR == 0 and etaR == 1 and muB==0):              
                muB = 1 - muB  
                etaR = 1 - etaR 

            #  First case accounted above: tagged in red zone w flag
            if (pointInCircle(zB[0:2], FR[0:2], gf) and qB == 0 and etaB == 0 and muR==1) or \
            (pointInCircle(zB[0:2], FB[0:2], gf) and qB == 0 and etaB == 1 and muR==0) or \
            (not pointInRectangle(zB[0:2], X) and qB == 0 and etaB == 1 and muR ==0): 
                if not pointInRectangle(zB[0:2], X):
                    qB = 1 - qB  
                etaB = 1 - etaB  
                muR = 1 - muR 

            RobotsB['mu'][-1]=muB
            RobotsR['mu'][-1]=muR
            CurRobotB.upstate(zB,tauB-dt,qB,etaB,ti)
        thetab = Angle_Control1(ti,CurRobotR,CurRobotB,zR,tauR,qR,etaR,muR,muB,FR,FB,r,b, RobotsR,RobotsB, 'r', i, dt)
        zR = robot_dynamics(zR, [speed, thetab],dt)
        # CurRobotR.step(zR,tauR,qR,etaR,ti)
        CurRobotR.step(zR,tauR-dt,qR,etaR,ti)
        RobotsR['mu'][-1]=muR
        RobotsB['mu'][-1]=muB  
    RobotsB['mu'][-1]=muB     
    RobotsR['mu'].append(muR)



## Plot
# Set up the figure and axis
fig, ax = plt.subplots(figsize=(8, 8)) # fig: figure object; ax: an arry of axes

ax.set_xlim(-85,85)
ax.set_ylim(-45,45)
ax.plot([0,0],[-40,40],linestyle="--",color='k',linewidth=4)
ax.set_aspect('equal')
ax.grid(True)
ax.set_axisbelow(True)
ax.set_xticks(np.arange(-80, 81, 20))
ax.set_yticks(np.arange(-40, 41, 20))
ax.set_xticklabels([])
ax.set_yticklabels([])
for spine in ax.spines.values():
    spine.set_linewidth(2)
ax.tick_params(top=True, right=True) 
ax.tick_params(direction='in',length=3,width=1.2)
rectparam = patches.Rectangle((-80, -40),160, 80,linewidth=4,edgecolor='black',facecolor='none')
ax.add_patch(rectparam)
circleB = plt.Circle((FB[0], FB[1]), gf, color='b', fill=False, linewidth=3, linestyle=(0, (2, 8))) # gb=10
ax.add_patch(circleB)
circleR = plt.Circle((FR[0], FR[1]), gf, color='r', fill=False, linewidth=3, linestyle=(0, (2, 8)))
ax.add_patch(circleR)
# set the size of field
ax.set(xlim=[x_f[0], x_f[2]], ylim=[y_f[0], y_f[2]])
# flag scatter
flag_scatter_B = ax.scatter(FB[0], FB[1], c="b", s=100, marker='x', linewidths=5)
flag_scatter_R = ax.scatter(FR[0], FR[1], c="r", s=100, marker='x', linewidths=5)



#Scatter Plots of original positions
robot_scatter_1B = ax.scatter(RobotsB["Robot1"].getTime(0)[0][0], RobotsB["Robot1"].getTime(0)[0][1], c="b", s=100)
rect_1B = rect_with_side_hinge(ax, (RobotsB["Robot1"].getTime(0)[0][0], RobotsB["Robot1"].getTime(0)[0][1]), RobotsB["Robot1"].getTime(0)[0][2], width=1, height=8,
                               side='bottom', radians=True, fc='blue')
robot_scatter_2B = ax.scatter(RobotsB["Robot2"].getTime(0)[0][0], RobotsB["Robot2"].getTime(0)[0][1], c="b", s=100)
rect_2B = rect_with_side_hinge(ax, (RobotsB["Robot2"].getTime(0)[0][0], RobotsB["Robot2"].getTime(0)[0][1]), RobotsB["Robot2"].getTime(0)[0][2], width=1, height=8,
                               side='bottom', radians=True, fc='blue')
robot_scatter_3B = ax.scatter(RobotsB["Robot3"].getTime(0)[0][0], RobotsB["Robot3"].getTime(0)[0][1], c="b", s=100)
rect_3B = rect_with_side_hinge(ax, (RobotsB["Robot3"].getTime(0)[0][0], RobotsB["Robot3"].getTime(0)[0][1]), RobotsB["Robot3"].getTime(0)[0][2], width=1, height=8,
                               side='bottom', radians=True, fc='blue')

robot_scatter_1R = ax.scatter(RobotsR["Robot1"].getTime(0)[0][0], RobotsR["Robot1"].getTime(0)[0][1], c="r", s=100)
rect_1R = rect_with_side_hinge(ax, (RobotsR["Robot1"].getTime(0)[0][0], RobotsR["Robot1"].getTime(0)[0][1]), RobotsR["Robot1"].getTime(0)[0][2], width=1, height=8,
                               side='bottom', radians=True, fc='red')
robot_scatter_2R = ax.scatter(RobotsR["Robot2"].getTime(0)[0][0], RobotsR["Robot2"].getTime(0)[0][1], c="r", s=100)
rect_2R = rect_with_side_hinge(ax, (RobotsR["Robot2"].getTime(0)[0][0], RobotsR["Robot2"].getTime(0)[0][1]), RobotsR["Robot2"].getTime(0)[0][2], width=1, height=8,
                               side='bottom', radians=True, fc='red')

robot_scatter_3R = ax.scatter(RobotsR["Robot3"].getTime(0)[0][0], RobotsR["Robot3"].getTime(0)[0][1], c="r", s=100)
rect_3R = rect_with_side_hinge(ax, (RobotsR["Robot3"].getTime(0)[0][0], RobotsR["Robot3"].getTime(0)[0][1]), RobotsR["Robot3"].getTime(0)[0][2], width=1, height=8,
                               side='bottom', radians=True, fc='red')

ax.add_patch(rect_1B)
ax.add_patch(rect_1R)
ax.add_patch(rect_2B)
ax.add_patch(rect_2R)
ax.add_patch(rect_3B)
ax.add_patch(rect_3R)


# Update function for each animation frame
def update(i):
    # Clear the current frame
    ax.clear()

    # Reset the axis limits
    ax.set_xlim(-85,85)
    ax.set_ylim(-45,45)

    # Draw center field line (midfield divider)
    ax.plot([0,0],[-40,40],linestyle="--",color='k',linewidth=4)

    # Set aspect ratio and labels
    ax.set_aspect('equal')

    ax.grid(True)
    ax.set_axisbelow(True)
    ax.set_xticks(np.arange(-80, 81, 20))
    ax.set_yticks(np.arange(-40, 41, 20))
    ax.set_xticklabels([])
    ax.set_yticklabels([]) 
    ax.tick_params(top=True, right=True) 
    ax.tick_params(direction='in',length=3,width=1.2)
    rectparam = patches.Rectangle((-80, -40),160, 80,linewidth=4,edgecolor='black',facecolor='none')
    ax.add_patch(rectparam)

    # Draw team B (blue) flag zone boundary as a circle
    circleB = plt.Circle((FB[0], FB[1]), gf, color='b', fill=False, linewidth=5, linestyle=(0,(2.5,2.5)))
    ax.add_patch(circleB)

    # Draw team R (red) flag zone boundary
    circleR = plt.Circle((FR[0], FR[1]), gf, color='r', fill=False, linewidth=5, linestyle=(0,(2.5,2.5)))
    ax.add_patch(circleR)

    # Scatter (plot) flags with dynamic color:
    # - If flag is held (mu = 1), show team color
    # - Else (mu = 0), show in black
    if RobotsB['mu'][i] == 1:
        flag_scatter_B = ax.scatter(FB[0], FB[1], c="b", s=100, marker='x', linewidths=5)
    else:
        flag_scatter_B = ax.scatter(FB[0], FB[1], c="k", s=0)

    if RobotsR['mu'][i] == 1:
        flag_scatter_R = ax.scatter(FR[0], FR[1], c="r", s=100, marker='x', linewidths=5)
    else:
        flag_scatter_R = ax.scatter(FR[0], FR[1], c="k", s=0)

    # Define list of all 6 robots with their team-specific colors and data sources
    robots = [
        # (name, base color, carry flag color, inactive color, scatter ref, rect ref, team data)
        ("Robot1", 'b', 'k', 'g', robot_scatter_1B, rect_1B, RobotsB),
        ("Robot2", 'b', 'k', 'g', robot_scatter_2B, rect_2B, RobotsB),
        ("Robot3", 'b', 'k', 'g', robot_scatter_3B, rect_3B, RobotsB),

        ("Robot1", 'r', 'k', 'orange', robot_scatter_1R, rect_1R, RobotsR),
        ("Robot2", 'r', 'k', 'orange', robot_scatter_2R, rect_2R, RobotsR),
        ("Robot3", 'r', 'k', 'orange', robot_scatter_3R, rect_3R, RobotsR)
    ]

    # Update each robot's scatter and rectangle (pose) for the current frame
    for robot_name, color_0, color_1, color_2, scatter_obj, rect_obj, robot_dict in robots:

        # Remove existing scatter object from plot (if it exists)
        if scatter_obj in ax.collections:
            scatter_obj.remove()

        # Get robot's position and heading angle from recorded data
        x = robot_dict[robot_name].getTime(i)[0][0]
        y = robot_dict[robot_name].getTime(i)[0][1]
        theta = (robot_dict[robot_name].getTime(i)[0][2]) / (2 * np.pi) * 360 - 90  # Convert rad to deg, offset for display

        # Set robot's orientation using rectangle patch
        update_rect_transform(ax,rect_obj,(x,y),theta,side='bottom',radians=False)
        ax.add_patch(rect_obj)

        # Determine robot's scatter marker type based on activation status
        eta = robot_dict[robot_name].getTime(i)[3]
        if eta == 0:
            scatter_obj = ax.scatter(x, y, s=100, marker='o', alpha=1)
        elif eta == 1:
            scatter_obj = ax.scatter(x, y, s=250, marker='*', alpha=1)
        else:
            scatter_obj = ax.scatter(x, y, s=250, marker='*', alpha=1)

        # Set new scatter location
        data = np.stack([x, y]).T
        scatter_obj.set_offsets(data)

        # Determine robot color based on flag and field position status
        z1 = robot_dict[robot_name].getTime(i)[1]  # x-position
        q = robot_dict[robot_name].getTime(i)[2]  # eta state

        if z1 < 0 and q == 0:
            scatter_obj.set_facecolor(color_0)  # Base color
        elif q == 1:
            scatter_obj.set_facecolor(color_1)  # Carrying flag
        else:
            scatter_obj.set_facecolor(color_2)  # Inactive/default

    # Return all scatter and rectangle objects for animation blitting
    return robot_scatter_1B, robot_scatter_1R, rect_1B, rect_1R, flag_scatter_B, flag_scatter_R, \
           robot_scatter_2B, rect_2B, robot_scatter_2B, robot_scatter_2R, \
           robot_scatter_3B, robot_scatter_3R, rect_3B, rect_3R

# Create the animation
anim = animation.FuncAnimation(fig,update,frames=len(RobotsB['Robot1'].getData()), interval=5)#(int(TSPAN[1]/dt)))

# Display the animation
plt.show()

#Save animation
#anim.save("animation50.gif",writer='pillow',fps=20)