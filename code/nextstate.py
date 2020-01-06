import modern_robotics as mr
import numpy as np

def NextState(curr_config,speed,time_step,speed_max):
	""" Compute the next time robot configuration

		Input:
		:param curr_config: A 12-vector representing the current configuration of the robot (3 variables for the 
				chassis configuration(x,y,phi), 5 variables for the arm configuration, and 4 variables 
				for the wheel angles).
		:param speed: A 9-vector of controls indicating the arm joint speeds thetadot (5 variables) and the 
				wheel speeds u (4 variables). 
		:param time_step: A timestep delta t. 
		:param speed_max: A positive real value indicating the maximum angular speed of the arm joints and the wheels.

		Output: 
		:param next_config: A 12-vector representing the configuration of the robot time deltat later. 
		:a list of 13 entries csv file (the 12-vector consisting of 3 chassis configuration variables, 
		the 5 arm joint angles, and the 4 wheel angles, plus a "0" for "gripper open") representing the robot's 
		configuration after each integration step

		Other param:
		:param det_u: u*time_step
		:param det_theta: thetadot*time_step

		Algorithm:
		The function NextState is based on a simple first-order Euler step, i.e.,

    	new arm joint angles = (old arm joint angles) + (joint speeds) * deltat
    	new wheel angles = (old wheel angles) + (wheel speeds) * deltat
    	new chassis configuration is obtained from odometry, as described in Chapter 13.4 

    	example input:

    	curr_config = [0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0]
    	speed = [2,2,2,2,2,10,10,10,10]
    	time_step = 0.01
    	speed_max = 12
	"""


    	# set variables
    	r = 0.0475
    	l = 0.235
    	w = 0.15
    	# get current configuration from input,qk is the configuration of chasis, jangle for joint angles and wangle for wheel angles
    	qk = np.array([[curr_config[0]],[curr_config[1]],[curr_config[2]]])
    	curr_jangle = np.array([[curr_config[3]],[curr_config[4]],[curr_config[5]],[curr_config[6]],[curr_config[7]]])
    	curr_wangle = np.array([[curr_config[8]],[curr_config[9]],[curr_config[10]],[curr_config[11]]])
    	# derive next joint configuration
    	theta_dot = np.array([[speed[0]],[speed[1]],[speed[2]],[speed[3]],[speed[4]]])
    	det_theta = theta_dot*time_step
    	next_jangle = curr_jangle + det_theta
    	# derive next wheel configuration
    	u = np.array([[speed[5]],[speed[6]],[speed[7]],[speed[8]]])
    	det_u = u*time_step
    	next_wangle = curr_wangle + det_u
    	# compute next chasis configuration
    	F = r/4 * np.array([[-1/(l+w),1/(l+w),1/(l+w),-1/(l+w)],[1,1,1,1],[-1,1,-1,1]])
    	print(F)
    	V_b = np.dot(F,det_u).reshape(3,)
    	wbz = V_b[0]
    	vbx = V_b[1]
    	vby = V_b[2]
    	det_qb = np.zeros(3,)
    	if wbz == 0.0:
    		det_qb = np.array([[0],[vbx],[vby]])
    	else:
    		det_qb = np.array([[wbz],[vbx*np.sin(wbz)+vby*(np.cos(wbz)-1)/wbz],[vby*np.sin(wbz)+vbx*(1-np.cos(wbz))/wbz]])
    		


    	update_matrix = np.array([[1,0,0],
    							[0,np.cos(curr_config[0]),-np.sin(curr_config[0])],
    							[0,np.sin(curr_config[0]),np.cos(curr_config[0])]])

    	det_q = np.dot(update_matrix,det_qb)
    	next_q = qk + det_q

 
    	# output the next configuration
    	next_config = np.concatenate((next_q, next_jangle,next_wangle), axis=None)
    	return next_config



# example input:

whole_traj = []
curr_config = np.array([0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0])
speed = np.array([0,0,0,0,0,-10,10,10,-10])
time_step = 0.01
speed_max = 12
next_config = NextState(curr_config,speed,time_step,speed_max)
print(next_config)

#derive csv file for 100 loops
current_traj = np.concatenate((curr_config,0),axis=None)
whole_traj.append(current_traj.tolist())
for i in range (100):
	next_config = NextState(curr_config,speed,time_step,speed_max)
	curr_config = next_config
	current_traj = np.concatenate((curr_config,0),axis=None)
	whole_traj.append(current_traj.tolist())

#append 0 at the end, output the desired csv file
np.savetxt('next_config.csv', whole_traj, delimiter=',')