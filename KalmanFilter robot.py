# -*- coding: utf-8 -*-
"""
Created on Fri Oct 26 11:42:26 2018

@author: Jay Jackman
"""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import math
import time
from scipy.linalg import block_diag
from scipy.linalg import sqrtm
from matplotlib.font_manager import FontProperties

class Robot:
    def __init__(self, x_pos, y_pos, theta, wheel_radius, axel_length):
        #Robot Constants
        #The particular part has a max speed of 130 RPM. So, we will convert to radians/ms
        self.MAX_RATE = 130 #in RPM 
        self.r = wheel_radius #in mm
        self.l = axel_length #in mm
        
        #state
        self.x = x_pos
        self.y = y_pos
        self.theta = theta
         
        #inputs are in RPM to comply with the datasheet/readability
        self.RPM_left = 0
        self.RPM_right = 0
        
        #real-world parameters (variances)
        #state noises
        self.sig_left_wheel = float(0.5**2)
        self.sig_right_wheel = float(0.5**2)
        self.sig_theta_s = 0.05**2 #not used
        
        #observation noises
        self.sig_front = float(0.07**2)
        self.sig_right = float(0.07**2)
        self.sig_theta_o = float(0.05**2)
        
        self.color = 'b'
    
    #This is the state propogation function.
    #It takes in a current state as well as the inputs, and returns the next state
    def getNextState(self, x,y,theta,inRight,inLeft):
        
        #need to calculate V_R and V_L in mm/cs (using centiseconds instead of milliseconds so visualizations are faster)
        V_L = (inRight + self.MAX_RATE*np.random.normal(loc=0,scale=self.sig_left_wheel)) * (2*math.pi*self.r)/6000.0
        V_R = (inLeft + self.MAX_RATE*np.random.normal(loc=0,scale=self.sig_right_wheel)) * (2*math.pi*self.r)/6000.0
        
        #get omega in radians/ms. This is the speed of rotation around the calculated center
        omega = (V_R-V_L)/self.l   

        if(omega == 0): #if we are moving straight
            next_x = V_L*math.sin(theta) + x
            next_y = V_L*math.cos(theta) + y
            next_theta = theta            
        else:
            #R is the distance from the center of robot to center of curvature
            R = (self.l/2.0)*(V_L+V_R)/(V_R-V_L)
            
            next_x = x + R*(math.cos(omega)*math.cos(theta) + math.sin(omega)*math.sin(theta) - math.cos(theta))
            next_y = y + R*(math.sin(omega)*math.cos(theta) - math.cos(omega)*math.sin(theta) + math.sin(theta))
            next_theta = float((theta - omega)) % (2*math.pi)

        return [next_x, next_y, next_theta]
        
        

class GameBoard:
    def __init__(self, width, length, robot):
        self.width = width #X dimension
        self.length = length #Y dimension
        self.robot = robot #holds the actual robot
        self.estimated_robot = Robot(0,0,0,wheel_radius=20, axel_length=85.0) #holds the estimated robot
        self.estimated_robot.color = 'g'
        
        self.fig = None 
        self.ax = None
        self.drawInitialize()
               
    def drawInitialize(self):
        
        #show the board itself
        x_lims = (0, self.width)
        y_lims = (0, self.length)
        self.fig = plt.figure(figsize=(5,5))
        self.ax = self.fig.add_subplot(111, aspect='equal')
        plt.xticks(np.arange(0, self.width+1, int(self.width/5)))
        plt.yticks(np.arange(0, self.length+1, int(self.length/5)))
        plt.xlim(x_lims)
        plt.ylim(y_lims)
        
        
        #print the robot
        robotPatch = self.getRobotPatch(self.robot)
        for i in range(len(robotPatch)):
            self.ax.add_patch(robotPatch[i])
        plt.show()
       

    
    def getLaserPatches(self):
        [d_front, d_right] = self.getSensorReadings()
        front_x = self.robot.x + d_front * math.sin(self.robot.theta)
        front_y = self.robot.y + d_front * math.cos(self.robot.theta)
        right_x = self.robot.x + d_right * math.sin(self.robot.theta + math.pi/2)
        right_y = self.robot.y + d_right * math.cos(self.robot.theta + math.pi/2)
        
        front_patch = patches.Arrow(self.robot.x, self.robot.y, front_x - self.robot.x, front_y - self.robot.y, width=20)
        front_patch.set_facecolor('b')
        right_patch = patches.Arrow(self.robot.x, self.robot.y, right_x - self.robot.x, right_y - self.robot.y, width=20)
        right_patch.set_facecolor('r')
        
        return [front_patch, right_patch]
    
    def draw(self):
        self.ax.patches = []
        self.fig.canvas.flush_events()
        
        robotPatch1 = self.getRobotPatch(self.robot)
        for i in range(len(robotPatch1)):
            self.ax.add_patch(robotPatch1[i])
            
        robotPatch2 = self.getRobotPatch(self.estimated_robot)
        for i in range(len(robotPatch2)):
            self.ax.add_patch(robotPatch2[i])
        
        
        #Add laser measurements
        [d_front, d_right] = self.getSensorReadings()
        front_x = self.robot.x + d_front * math.sin(self.robot.theta)
        front_y = self.robot.y + d_front * math.cos(self.robot.theta)
        right_x = self.robot.x + d_right * math.sin(self.robot.theta + math.pi/2)
        right_y = self.robot.y + d_right * math.cos(self.robot.theta + math.pi/2)
        
        front_patch = patches.Arrow(self.robot.x,self.robot.y, front_x - self.robot.x, front_y - self.robot.y, width=20)
        front_patch.set_facecolor('b')
        right_patch = patches.Arrow(self.robot.x,self.robot.y, right_x - self.robot.x, right_y - self.robot.y, width=20)
        right_patch.set_facecolor('r')
        self.ax.add_patch(front_patch)
        self.ax.add_patch(right_patch)
        
        self.fig.canvas.draw()
        plt.show()
        
        
    #returns a patch array to draw the robot's current position
    def getRobotPatch(self, robot):        
        cos = math.cos(robot.theta)
        sin = math.sin(robot.theta)
        r = robot.r
        length = robot.l
        x = robot.x
        y = robot.y
        
        #centers of the wheels
        LW_x = -length/2.0*cos + x
        LW_y = length/2.0*sin + y
        RW_x = length/2.0*cos + x
        RW_y = -length/2.0*sin + y
        
        #front and back edges of the left wheel
        LW_front_x = r*sin + LW_x
        LW_front_y = r*cos + LW_y
        LW_back_x = -r*sin + LW_x
        LW_back_y = -r*cos + LW_y
        
        #front and back edges of the right wheel
        RW_front_x = r*sin + RW_x
        RW_front_y = r*cos + RW_y
        RW_back_x = -r*sin + RW_x
        RW_back_y = -r*cos + RW_y
        
        #used for painting a direction arrow
        #x_head = max(self.robot.RPM_left, self.robot.RPM_right)/130*75*sin + x
        #y_head = max(self.robot.RPM_left, self.robot.RPM_right)/130*75*cos + y
#        x_head = length/130*75*sin + x
#        y_head = length/130*75*cos + y
            
        #four corners of the body of the robot
        UL_x = -0.70*length/2.0*cos + x
        UL_y = 0.70*length/2.0*sin + y
        LL_x = -length*sin + UL_x
        LL_y = -length*cos + UL_y
        UR_x = 0.70*length/2.0*cos + x
        UR_y = -0.70*length/2.0*sin + y
        LR_x = -length*sin + UR_x
        LR_y = -length*cos + UR_y
        
        #create the body patch
        bodyArray = np.array([[UL_x,UL_y],[LL_x,LL_y],[LR_x,LR_y],[UR_x,UR_y]])        
        body = patches.Polygon(bodyArray)
        body.set_edgecolor('black')
        body.set_linewidth(3)
        body.set_facecolor(robot.color)
        
        #create the axel
        axel = patches.FancyArrowPatch((LW_x, LW_y),(RW_x,RW_y))
        
        #create the left wheel
        left_wheel = patches.FancyArrowPatch((LW_front_x, LW_front_y),(LW_back_x, LW_back_y))
        left_wheel.set_linewidth(5)
        
        #create the right wheel
        right_wheel = patches.FancyArrowPatch((RW_front_x, RW_front_y),(RW_back_x, RW_back_y))
        right_wheel.set_linewidth(5)
        
        #direction_arrow = patches.FancyArrowPatch((robot.x, robot.y), (x_head,y_head), mutation_scale=20)
        
        return [axel, left_wheel, right_wheel, body]
    
    #returns [DistanceFront, DistanceRight]
    def getSensorReadings(self):
        dist_front = self.getDistance(self.robot.x, self.robot.y, self.robot.theta)
        dist_right = self.getDistance(self.robot.x, self.robot.y, float(self.robot.theta + math.pi/2) % (2*math.pi))
        return [dist_front, dist_right]
    
    def getTheta(self,theta):
        return float(theta + 2*math.pi*np.random.normal(loc=0,scale=(self.robot.sig_theta_o))) % (2*math.pi)
    
    def getDistance(self, x, y, theta):
        #check if robot is facing straight up
        if(theta == 0):
            intersection_x = x
            intersection_y = self.length
        #upper-right quadrant
        elif(theta < math.pi/2):
            slope = 1/math.tan(theta)
            temp_x = (self.length - y)/slope + x #find intersection with top line
            temp_y = slope*(self.width - x) + y #find intersection with right line
            if(temp_x > self.width):
                intersection_x = self.width
                intersection_y = temp_y
            else:
                intersection_x = temp_x
                intersection_y = self.length
        #facing right
        elif(theta == math.pi/2):
            intersection_x = self.width
            intersection_y = y
        #lower-right quadrant
        elif(theta < math.pi):
            slope = 1/math.tan(theta)
            temp_x = -y/slope + x #find intersection with bottom line
            temp_y = slope*(self.width - x) + y #find intersection with right line
            if(temp_x > self.width):
                intersection_x = self.width
                intersection_y = temp_y
            else:
                intersection_x = temp_x
                intersection_y = 0
        #pointing down
        elif(theta == math.pi):
            intersection_x = x
            intersection_y = 0
        #lower-left quadrant
        elif(theta < 3*math.pi/2):
            slope = 1/math.tan(theta)
            temp_x = -y/slope + x #find intersection wtih bottom line
            temp_y = slope*(-x) + y #find intersection with left line
            if(temp_x < 0):
                intersection_x = 0
                intersection_y = temp_y
            else:
                intersection_x = temp_x
                intersection_y = 0
        #facing left
        elif(theta == 3*math.pi/2):
            intersection_x = 0
            intersection_y = y
        #upper-left quadrant
        else:
            slope = 1/math.tan(theta)
            temp_x = (self.length - y)/slope + x #find intersection with top line
            temp_y = slope*(-x) + y #find intersection with left line
            if(temp_x < 0):
                intersection_x = 0
                intersection_y = temp_y
            else:
                intersection_x = temp_x
                intersection_y = self.length
        
        true_distance = math.sqrt((intersection_x - x)**2 + (intersection_y - y)**2)
        observed_distance = true_distance + true_distance*np.random.normal(0,self.robot.sig_front)
        
        return(observed_distance)
        
        #return math.sqrt((intersection_x - x)**2 + (intersection_y - y)**2)
    
    def run(self, inputs, withSmoothing = True, initialKnown = True, withAnimation=True):
        n=8
        num_steps = inputs.shape[1]
        
        #hold the history of the robots for plotting       
        true_states = np.zeros([3,num_steps]) #holds the true position of the robot
        true_observations = np.zeros([3,num_steps]) #holds the true observations of the robot
        estimated_states = np.zeros([3,num_steps]) #holds the estimated position of the robot
        estimated_observations = np.zeros([3,num_steps]) #holds the estimated observations of the robot
               
        #INITIALIZE KALMAN FILTER
        if(initialKnown):
            x_k = np.array([[self.robot.x],[self.robot.y],[self.robot.theta]])
        else:
            x_k = np.array([[np.random.uniform(0,self.width)],[np.random.uniform(0,self.length)],[np.random.uniform(0,math.pi*2)]]) 
        
        if(initialKnown):
            P_xk = block_diag(0.001,0.001,0.001)
        else:
            P_xk = block_diag(1,1,1)
        P_v = block_diag(self.robot.sig_left_wheel, self.robot.sig_right_wheel)
        P_n = block_diag(self.robot.sig_front, self.robot.sig_right, self.robot.sig_theta_o)
        P_k = block_diag(P_xk,P_v,P_n)
                        
        W_0 = .5
        W = (1-W_0)/(2*n)
        coeff = n/(1-W_0)
        
        #START UNSCENTED KALMAN FILTER (refer to PDF for algorithm details)
        for k in range(num_steps):
            x_k_prev = x_k #only used for smoothing
            
            #CALCULATE SIGMA POINTS
            S = sqrtm(P_k*coeff)
            x_k = np.vstack((x_k, np.array([[0],[0]]), np.array([[0],[0],[0]])))
            sigmas = [x_k]
            for i in range(n):
                sigmas.append(x_k + S[:,i:i+1])
            for i in range(n):
                sigmas.append(x_k - S[:,i:i+1])
            X_k = np.hstack(sigmas)
            X_k_x = X_k[0:3,:]
            
            #TIME UPDATE
            
            #Transform the Sigma Points
            propogated_points = []
            for i in range(len(sigmas)):
                x = X_k_x[0,i]
                y = X_k_x[1,i]
                theta = X_k_x[2,i]
                inRight = self.robot.RPM_right
                inLeft = self.robot.RPM_left
                [tempx,tempy,temptheta] = self.robot.getNextState(x,y,theta,inRight,inLeft)
                temp = np.array([[tempx],[tempy],[temptheta]])
                propogated_points.append(temp)
            X_k_next = np.hstack(propogated_points)
            
            #Calculate the a-priori estimate of the state
            x_k = W_0 * X_k_next[:,0:1]
            for i in range(2*n):
                x_k = x_k + (W * X_k_next[:,i+1:i+2])
            
            #Calculate the a-priori estimate of the covariance
            vec = X_k_next[:,0:1] - x_k
            P_xk = W_0 * np.matmul(vec,vec.T)
            for i in range(2*n):
                vec = X_k_next[:,i+1:i+2] - x_k
                P_xk = P_xk + W*np.matmul(vec,vec.T)
            
            #MEASUREMENT UPDATE
            
            #Transform the Sigma Points
            observed_points = []
            for i in range(len(sigmas)):
                x = X_k_next[0,i]
                y = X_k_next[1,i]
                theta = X_k_next[2,i]
                observed_front = self.getDistance(x,y,theta)
                observed_right = self.getDistance(x,y,float((theta+math.pi/2))%(2*math.pi))
                observed_theta = self.getTheta(theta)
                observed_points.append(np.array([[observed_front],[observed_right],[observed_theta]]))
            Y_k = np.hstack(observed_points)
            
            #Calculate the estimated measurement vector
            y_k = W_0 * Y_k[:,0:1]
            for i in range(2*n):
                y_k = y_k + (W * Y_k[:,i+1:i+2])
            
            #Calculate the estimated measurement covariance
            P_yk = W_0 * np.matmul((Y_k[:,0:1]-y_k),(Y_k[:,0:1]-y_k).T)
            for i in range(2*n):
                vec = Y_k[:,i+1:i+2] - y_k
                P_yk = P_yk + W*np.matmul(vec,vec.T)
            
            #Calculate the Cross Covariance
            P_xkyk = W_0 * np.matmul((X_k_next[:,0:1]-x_k), (Y_k[:,0:1]-y_k).T)
            for i in range(2*n):
                xvec = X_k_next[:,i+1:i+2] - x_k
                yvec = Y_k[:,i+1:i+2] - y_k
                P_xkyk = P_xkyk + W*np.matmul(xvec,yvec.T)
            
            #Calculate the Kalman Gain
            K_k = np.matmul(P_xkyk, np.linalg.inv(P_yk))
            
            #Get the actual observations
            read_front = self.getDistance(self.robot.x, self.robot.y, self.robot.theta)
            read_right = self.getDistance(self.robot.x, self.robot.y, (self.robot.theta+math.pi/2)%(2*math.pi))
            read_theta = self.getTheta(self.robot.theta)
            y_k_actual = np.array([[read_front],[read_right],[read_theta]])
            
            #Use the innovation to update the estimated state and covariance
            x_k = x_k + np.matmul(K_k, (y_k_actual-y_k))
            P_xk = P_xk - np.matmul(K_k, np.matmul(P_yk, K_k.T))
            P_k = block_diag(P_xk,P_v,P_n)
            
            #END UKF
           
            #Apply custom smoothing algorithm
            if(withSmoothing):
                tempx = x_k[0]
                tempy = x_k[1]
                if(tempx < 0 or tempx > self.width or tempy < 0 or tempy > self.length):
                    x_k[0:2] = x_k_prev[0:2]
                    self.estimated_robot.color = 'orange'
                else:
                    self.estimated_robot.color = 'g'
            
            #Update the state history for tracking
            true_states[0,k] = self.robot.x
            true_states[1,k] = self.robot.y
            true_states[2,k] = self.robot.theta
            true_observations[0,k] = read_front
            true_observations[1,k] = read_right
            true_observations[2,k] = read_theta
            estimated_states[0,k] = x_k[0].real
            estimated_states[1,k] = x_k[1].real
            estimated_states[2,k] = x_k[2].real
            estimated_observations[0,k] = y_k[0].real
            estimated_observations[1,k] = y_k[1].real
            estimated_observations[2,k] = y_k[2].real
                            
            self.estimated_robot.x = float(x_k[0].real)
            self.estimated_robot.y = float(x_k[1].real)
            self.estimated_robot.theta = float(x_k[2].real)
            
            if(withAnimation):
                self.draw()
            
            #propogate actual robot
            self.robot.RPM_left = inputs[0,k]
            self.robot.RPM_right = inputs[1,k]
            [x,y,theta] = self.robot.getNextState(self.robot.x,self.robot.y,self.robot.theta,self.robot.RPM_right,self.robot.RPM_left)
            self.robot.x = x
            self.robot.y = y
            self.robot.theta = theta
        plt.close('all')    
        return [true_states, true_observations, estimated_states, estimated_observations]

def generateInputs():
    left_inputs = []
    right_inputs = []
    num_steps = 750
    for i in range(100):
        left_inputs.append(0)
        right_inputs.append(0)
    for i in range(200):
        left_inputs.append(130)
        right_inputs.append(-130)
    for i in range(num_steps):
        k = i/num_steps*130
        left_inputs.append(20)
        right_inputs.append(k)    
    for i in range(50):
        left_inputs.append(-60)
        right_inputs.append(-130)
    for i in range(50):
        left_inputs.append(-130)
        right_inputs.append(-60)
    for i in range(50):
        left_inputs.append(60)
        right_inputs.append(-130)
    for i in range(50):
        left_inputs.append(-130)
        right_inputs.append(60)
    for i in range(150):
        left_inputs.append(-130)
        right_inputs.append(130)
    for i in range(150):
        left_inputs.append(0)
        right_inputs.append(130)
    return np.vstack([left_inputs,right_inputs])


plt.close('all')

WIDTH = 750
LENGTH = 500
r = 20
l = 85
initial_x = np.random.uniform(0,WIDTH)
initial_y = np.random.uniform(0,LENGTH)
initial_theta = np.random.uniform(0,math.pi*2)

gb = GameBoard(WIDTH,LENGTH, Robot(500,300,3*math.pi/2, wheel_radius=r, axel_length=l))
time.sleep(0.5)
inputs = generateInputs()
[true_state, true_observations, estimated_states, estimated_observations] = gb.run(inputs,withSmoothing=True, initialKnown=False, withAnimation=True)

dims = (4,6)
fig = plt.figure(figsize=(15,15))
fig.suptitle('Initial Position Unknown, Inaccurate Magnetometer')
fontP = FontProperties()
fontP.set_size('small')

ax_position_plot = plt.subplot2grid(dims, (0,0),rowspan=2, colspan=3)
box = ax_position_plot.get_position()
ax_position_plot.set_position([box.x0+box.width*0.1, box.y0+box.height*0.1, box.width*0.9, box.height*0.9])
plt.title('Esitamated and true Positions')
x_lims = (0, WIDTH)
y_lims = (0, LENGTH)
plt.xticks(np.arange(0, WIDTH+1, int(WIDTH/5)))
plt.yticks(np.arange(0, LENGTH+1, int(LENGTH/5)))
plt.xlim(x_lims)
plt.ylim(y_lims)
plt.plot(true_state[0,:],true_state[1,:], 'b-')
plt.plot(estimated_states[0,:], estimated_states[1,:], 'g-')
plt.xlabel('x-Position')
plt.ylabel('y-Position')
plt.legend(['True Position', 'Estimated Position'])

ax_errors = plt.subplot2grid(dims,(0,3), rowspan=2,colspan=3)
box = ax_errors.get_position()
ax_errors.set_position([box.x0+box.width*0.1, box.y0+box.height*0.1, box.width*0.9, box.height*0.9])
plt.title('Position Euclidian Errors')
diffx = true_state[0,:] - estimated_states[0,:]
diffy = true_state[1,:] - estimated_states[1,:]
squaresx = [i**2 for i in diffx]
squaresy = [i**2 for i in diffy]
sums = squaresx + squaresy
errors = [math.sqrt(i) for i in sums]
plt.plot(errors)
plt.xlabel('Time')
plt.ylabel('Error (mm)')

ax_distance_observations = plt.subplot2grid(dims,(2,0),rowspan=2, colspan=3)
box = ax_distance_observations.get_position()
ax_distance_observations.set_position([box.x0+box.width*0.1, box.y0+box.height*0.1, box.width*0.9, box.height*0.9])
plt.title('Estimated and True Distance Observations')
plt.plot(true_observations[0,:])
plt.plot(estimated_observations[0,:])
plt.plot(true_observations[1,:])
plt.plot(estimated_observations[1,:])

ax_angle_observations = plt.subplot2grid(dims,(2,3), rowspan=2, colspan=3)
box = ax_angle_observations.get_position()
ax_angle_observations.set_position([box.x0+box.width*0.1, box.y0+box.height*0.1, box.width*0.9, box.height*0.9])
plt.title('Estimated and True ' + r'$\theta$' + ' angle Observations')
plt.plot(true_observations[2,:], 'b')
plt.plot(estimated_observations[2,:], 'g')
plt.legend(['True ' + r'$\theta$', 'Estimated Observed ' + r'$\theta$'], prop=fontP)
plt.xlabel('Time')
plt.ylabel(r'$\theta$')

