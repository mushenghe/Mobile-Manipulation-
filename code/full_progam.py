import modern_robotics as mr
import numpy as np
import matplotlib.pyplot as plt
import logging

# initilize log file
LOG_FILENAME = 'newtask.log'
logging.basicConfig(filename=LOG_FILENAME,level=logging.DEBUG)

# this python file description:

# Since I've already add all the milestone functions in this file and add all the input I need, 
# just run "python full_progran.py" is enough to get the required csv output.


""" This full_program plans a trajectory for the end-effector of the youBot mobile manipulator 
	(a mobile base with four mecanum wheels and a 5R robot arm), performs odometry as the chassis 
	moves, and performs feedback control to drive the youBot to pick up a block at a specified location, 
	carry it to a desired location, and put it down. 

	+ Input:

		:param Tsc_ini: The cube's initial configuration
		:param Tsc_fin: The cube's desired final configuration
		:param youbot_actual: The actual initial configuration of the youBot 
		:param Kp,Ki: feed back controller gains

	+Other Parameters needed:
		:param Tse_ini: The initial configuration of the end-effector in the reference trajectory
		:param Tse_finish: The configuration of the end-effector in the reference trajectory when the 
						   cube arrives its final configuration 
		:param Tse_fin = The final configuration of the end-effector in the reference trajectory
		:param Tse_sta = The end-effector's configuration relative to the world frame when it is grasping 
						 the cube
		:param Tse_grp = The end-effector's configuration relative to the world frame when it is grasping 
						  the cube
		:param Tsc_ini: The cube's initial configuration
		:param Tsc_fin: The cube's desired final configuration
		:param Tce_grp: The end-effector's configuration relative to the cube when it is grasping the cube
		:param Tce_sta: The end-effector's standoff configuration above the cube, before and after grasping, 
						relative to the cube
		:param Tf: Total time of the motion in seconds from rest to rest
		:param k: The number of trajectory reference configurations per 0.01 seconds(1)
	
	+ Output: 

		: A list of 1400 * 13 entries csv file. The first 12 entries of each row are top three rows of the 
		transformation matrix Tse at that instant of time in SE(3), the last entry of each row is the gripper 
		state: 0 = open, 1 = closed.
		: A list of 1399 * 6 entries csv file: Xerr
		: A log file


"""
def matrix2list(whole_traj,endeffector_traj,N,gripper_state):
	'''
	This function convert the transformation matrix to a N * 13 entries list.
	'''
	sub = np.zeros((N,13),dtype = float)
	for i in range(N):
		sub[i][0] = endeffector_traj[i][0][0]
		sub[i][1] = endeffector_traj[i][0][1]
		sub[i][2] = endeffector_traj[i][0][2]
		sub[i][3] = endeffector_traj[i][1][0]
		sub[i][4] = endeffector_traj[i][1][1]
		sub[i][5] = endeffector_traj[i][1][2]
		sub[i][6] = endeffector_traj[i][2][0]
		sub[i][7] = endeffector_traj[i][2][1]
		sub[i][8] = endeffector_traj[i][2][2]
		sub[i][9] = endeffector_traj[i][0][3]
		sub[i][10] = endeffector_traj[i][1][3]
		sub[i][11] = endeffector_traj[i][2][3]
		sub[i][12] = gripper_state
		whole_traj.append(sub[i].tolist())
	
	return whole_traj

def NextState(curr_config,speed,time_step,speed_max):
	""" Compute the robot configuration delta t time later

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
    	V_b = np.dot(F,det_u).reshape(3,)
    	wbz = V_b[0]
    	vbx = V_b[1]
    	vby = V_b[2]
    	det_qb = np.zeros(3,)

    	if wbz < 1e-3:
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

def GenerateTrajectory(Tse_ini, Tsc_ini, Tsc_fin, Tce_grp, Tce_sta, k):
		"""Computes a trajectory as a list of N SE(3) matrices corresponding to
		   the screw motion about a space screw axis

		Input:
		:param Tse_ini: The initial configuration of the end-effector in the reference trajectory
		:param Tse_finish: The configuration of the end-effector in the reference trajectory when the cube arrives its final configuration 
		:param Tse_fin = The final configuration of the end-effector in the reference trajectory
		:param Tse_sta = The end-effector's configuration relative to the world frame when it is grasping the cube
		:param Tse_grp = The end-effector's configuration relative to the world frame when it is grasping the cube
		:param Tsc_ini: The cube's initial configuration
		:param Tsc_fin: The cube's desired final configuration
		:param Tce_grp: The end-effector's configuration relative to the cube when it is grasping the cube
		:param Tce_sta: The end-effector's standoff configuration above the cube, before and after grasping, relative to the cube
		:param Tf: Total time of the motion in seconds from rest to rest
		:param k: The number of trajectory reference configurations per 0.01 seconds(1)
	
		Return: 
		a list of 13 entries csv file:
				r11, r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz, gripper state
		
		"""

		whole_traj = []

		# 1st step:
		Tse_sta = np.dot(Tsc_ini,Tce_sta)
		Tse_grp = np.dot(Tsc_ini,Tce_grp)
		Tse_finish = np.dot(Tsc_fin,Tce_sta)
		Tse_fin = np.dot(Tsc_fin,Tce_grp)
		k = 1
		Tf = 3
		N = 300
		method = 5
		gripper_state = 0
		traj1 = np.asarray(mr.ScrewTrajectory(Tse_ini, Tse_sta, Tf, N, method))
		whole_traj = matrix2list(whole_traj,traj1,N,gripper_state)

		#2nd step:
		N = 100
		traj2 = np.asarray(mr.ScrewTrajectory(Tse_sta, Tse_grp, Tf, N, method))
		whole_traj = matrix2list(whole_traj,traj2,N,gripper_state)

		# 3rd step
		N = 100
		gripper_state = 1
		traj3 = np.asarray(mr.ScrewTrajectory(Tse_grp, Tse_grp, Tf, N, method))
		whole_traj = matrix2list(whole_traj,traj3,N,gripper_state)

		#4th step
		N = 100
		traj4 = np.asarray(mr.ScrewTrajectory(Tse_grp, Tse_sta, Tf, N, method))
		whole_traj = matrix2list(whole_traj,traj4,N,gripper_state)

		#5th step
		N = 500
		traj5 = np.asarray(mr.ScrewTrajectory(Tse_sta, Tse_finish, Tf, N, method))
		whole_traj = matrix2list(whole_traj,traj5,N,gripper_state)

		#6th step
		N = 100
		traj6 = np.asarray(mr.ScrewTrajectory(Tse_finish, Tse_fin, Tf, N, method))
		whole_traj = matrix2list(whole_traj,traj6,N,gripper_state)

		#7th step
		N = 100
		gripper_state = 0
		traj7 = np.asarray(mr.ScrewTrajectory(Tse_fin, Tse_fin, Tf, N, method))
		whole_traj = matrix2list(whole_traj,traj7,N,gripper_state)

		#8th step
		N = 100
		traj8 = np.asarray(mr.ScrewTrajectory(Tse_fin, Tse_finish, Tf, N, method))
		whole_traj = matrix2list(whole_traj,traj8,N,gripper_state)

		return whole_traj

def FeedbackControl(integral,robo_config,X,Xd,Xd_next,KP,KI,time_step):
	"""calculate the kinematic task-space feedforward plus feedback control law

		Input:
		:param X: The current actual end-effector configuration
		:param Xd: The current end-effector reference configuration
		:param Xd_next: The end-effector reference configuration at the next timestep in the reference trajectory
		:param Kp: the P gain matrix
		:param Ki: the I gain matrix
		:param time_step: A timestep delta t between reference trajectory configurations


		Output: 
		:param V: The commanded end-effector twist expressed in the end-effector frame
		:a list of 13 entries csv file (the 12-vector consisting of 3 chassis configuration variables, 
		the 5 arm joint angles, and the 4 wheel angles, plus a "0" for "gripper open") representing the robot's 
		configuration after each integration step

		Other param:
		:param thetalist: A list of joint coordinates
		:param Blist: The joint screw axes in the end-effector frame when the
                  		manipulator is at the home position
	"""
	r = 0.0475
	l = 0.235
	w = 0.15
	Tb0 = np.array([[ 1, 0, 0, 0.1662],
				    [ 0, 1, 0,   0],
				    [ 0, 0, 1, 0.0026],
				    [ 0, 0, 0,   1]])

	M0e = np.array([[ 1, 0, 0, 0.033],
				    [ 0, 1, 0,   0],
				    [ 0, 0, 1, 0.6546],
				    [ 0, 0, 0,   1]])
	
	Blist = np.array([[0, 0, 1,   0, 0.033, 0],
                      [0,-1, 0,-0.5076,  0, 0],
                      [0,-1, 0,-0.3526,  0, 0],
                      [0,-1, 0,-0.2176,  0, 0],
                      [0, 0, 1,   0,     0, 0]]).T
	
	thetalist = robo_config[3:8]
	F = r/4 * np.array([[0,0,0,0],[0,0,0,0],[-1/(l+w),1/(l+w),1/(l+w),-1/(l+w)],[1,1,1,1],[-1,1,-1,1],[0,0,0,0]])
	T0e = mr.FKinBody(M0e,Blist,thetalist)
	Vd =  mr.se3ToVec((1/time_step)*mr.MatrixLog6(np.dot(mr.TransInv(Xd),Xd_next)))
	ADxxd = mr.Adjoint(np.dot(mr.TransInv(X),Xd))
	ADxxdVd = np.dot(ADxxd,Vd)
	Xerr = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(X),Xd)))
	V = ADxxdVd + np.dot(KP,Xerr) +  np.dot(KI,integral + Xerr * time_step)
	integral +=  Xerr * time_step
	Ja = mr.JacobianBody(Blist, thetalist)
	Jb = mr.Adjoint(np.dot(mr.TransInv(T0e),mr.TransInv(Tb0))).dot(F)
	Je = np.concatenate((Jb,Ja),axis=1)
	Je_inv = np.linalg.pinv(Je)
	command = Je_inv.dot(V)
	return command,Xerr

# MAIN START
# example input are listed as follows:
# set up variables:
def full_program(Tsc_ini,Tsc_fin,KP,KI,robo_config):
	youbot_traj = []
	Xerr_array = []
# use global variable to keep track of the change of integral
	global integral
	integral = np.zeros((6,),dtype = float)
# set up the initial variables
	Tse_ini = np.array([[ 0, 0, 1,   0],
					[ 0, 1, 0,   0],
					[ -1, 0,0, 0.5],
					[ 0, 0, 0,   1]])


	Tce_grp = np.array([[ -np.sqrt(2)/2, 0, np.sqrt(2)/2, 0],
					[ 0, 1, 0, 0],
					[-np.sqrt(2)/2, 0, -np.sqrt(2)/2, 0],
					[ 0, 0, 0, 1]])

	Tce_sta = np.array([[ -np.sqrt(2)/2, 0, np.sqrt(2)/2, 0],
					[ 0, 1, 0, 0],
					[-np.sqrt(2)/2, 0, -np.sqrt(2)/2, 0.1],
					[ 0, 0, 0, 1]])
	Tb0 = np.array([[ 1, 0, 0, 0.1662],
				    [ 0, 1, 0,   0],
				    [ 0, 0, 1, 0.0026],
				    [ 0, 0, 0,   1]])
	Blist = np.array([[0, 0, 1,   0, 0.033, 0],
                      [0,-1, 0,-0.5076,  0, 0],
                      [0,-1, 0,-0.3526,  0, 0],
                      [0,-1, 0,-0.2176,  0, 0],
                      [0, 0, 1,   0,     0, 0]]).T
	M0e = np.array([[ 1, 0, 0, 0.033],
				    [ 0, 1, 0,   0],
				    [ 0, 0, 1, 0.6546],
				    [ 0, 0, 0,   1]])
	k = 1
	speed_max = 12
	time_step = 0.01

	# first call the GenerateTrajectory function to get teh desired end-effector configuration of youbot
	traj = np.asarray(GenerateTrajectory(Tse_ini, Tsc_ini, Tsc_fin, Tce_grp, Tce_sta, k))
	# append the initial configuration to the whole trajectory
	youbot_traj.append(robo_config.tolist())
	# begin the loop
	for i in range (1399):
		# joint angle
		# every time update variables 
		thetalist = robo_config[3:8]		
		Xd = np.array([[ traj[i][0], traj[i][1], traj[i][2],  traj[i][9]],
				   	   [ traj[i][3], traj[i][4], traj[i][5], traj[i][10]], 
				       [ traj[i][6], traj[i][7], traj[i][8], traj[i][11]],
				       [          0,          0,          0,          1]])
		Xd_next = np.array([[ traj[i+1][0], traj[i+1][1], traj[i+1][2],  traj[i+1][9]],
				   	   		[ traj[i+1][3], traj[i+1][4], traj[i+1][5], traj[i+1][10]],
				      	    [ traj[i+1][6], traj[i+1][7], traj[i+1][8], traj[i+1][11]],
					        [            0,            0,            0,           1]])
		Tsb = np.array([[np.cos(robo_config[0]),-np.sin(robo_config[0]), 0, robo_config[1]],
				   	    [np.sin(robo_config[0]), np.cos(robo_config[0]), 0, robo_config[2]], 
				        [              0       ,          0            , 1,        0.0963 ],
				        [              0       ,          0            , 0,             1]])
		T0e = mr.FKinBody(M0e,Blist,thetalist)
		X = np.dot(Tsb,np.dot(Tb0,T0e))
		# get the command and error vector from feedback control
		command,Xerr = FeedbackControl(integral,robo_config,X,Xd,Xd_next,KP,KI,time_step)
		Xerr_array.append(Xerr.tolist())
		Cw = command[:4]
		Cj = command[4:9]
		# the input command of NextState and the command returned by FeedbackControl is flipped
		controls = np.concatenate((Cj,Cw),axis=None)
		robo_config = NextState(robo_config[:12],controls,time_step,speed_max)
		youbot_current_traj = np.concatenate((robo_config,traj[i][12]),axis=None)
		youbot_traj.append(youbot_current_traj.tolist())

	# save the Xerr vector
	logging.debug('generating Xerr data file')
	np.savetxt('Xerrnewtask.csv', Xerr_array, delimiter=',')
	# plot the Xerr
	logging.debug('plotting error data')
	qvec = np.asarray(Xerr_array)
	tvec = np.linspace(0,13.99,1399)
	plt.plot(tvec,qvec[:,0])
	plt.plot(tvec,qvec[:,1])
	plt.plot(tvec,qvec[:,2])
	plt.plot(tvec,qvec[:,3])
	plt.plot(tvec,qvec[:,4])
	plt.plot(tvec,qvec[:,5])
	plt.xlim([0,14])
	plt.title(' Xerr plot')
	plt.xlabel('Time (s)')
	plt.ylabel('error')
	plt.legend([r'$Xerr[1]$',r'$Xerr[2]$',r'$Xerr[3]$',r'$Xerr[4]$',r'$Xerr[5]$',r'$Xerr[6]$'])
	plt.grid(True)
	plt.show()

	logging.debug('generating animation csv file')
	np.savetxt('V-REP.csv', youbot_traj, delimiter=',')
	logging.debug('Done')

# example input	
kp = 50
ki = 15
Tsc_ini = np.array([[1, 0, 0,     1],
					[0, 1, 0,     0.5],
					[0, 0, 1, 0.025],
					[0, 0, 0,     1]])

Tsc_fin = np.array([[ 0, 1, 0,     0.2],
					[-1, 0, 0,    -2],
					[ 0, 0, 1, 0.025],
					[ 0, 0, 0,     1]])
KP = np.array([[kp, 0, 0, 0, 0, 0],
			   [ 0,kp, 0, 0, 0, 0],
			   [ 0, 0,kp, 0, 0, 0],
			   [ 0, 0, 0,kp, 0, 0],
			   [ 0, 0, 0, 0,kp, 0],
			   [ 0, 0, 0, 0, 0,kp]])
KI = np.array([[ki, 0, 0, 0, 0, 0],
			   [ 0,ki, 0, 0, 0, 0],
			   [ 0, 0,ki, 0, 0, 0],
			   [ 0, 0, 0,ki, 0, 0],
			   [ 0, 0, 0, 0,ki, 0],
			   [ 0, 0, 0, 0, 0,ki]])
# set up initial configuration of youbot
robo_config = np.array([0.1,0.1,0.2,0,0,0.2,-1.6, 0,0,0,0,0,0])
# call the function:
full_program(Tsc_ini,Tsc_fin,KP,KI,robo_config)