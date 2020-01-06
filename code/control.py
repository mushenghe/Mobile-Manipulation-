import modern_robotics as mr
import numpy as np

def FeedbackControl(X,Xd,Xd_next,Kp,Ki,time_step):
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
    	speed_max = 12
	"""
	r = 0.0475
	l = 0.235
	w = 0.15
	robo_config = np.array([0,0,0,0,0,0.2,-1.6, 0])
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
	
	thetalist = robo_config[3:]
	F = r/4 * np.array([[0,0,0,0],[0,0,0,0],[-1/(l+w),1/(l+w),1/(l+w),-1/(l+w)],[1,1,1,1],[-1,1,-1,1],[0,0,0,0]])
	T0e = mr.FKinBody(M0e,Blist,thetalist)
	Vd =  mr.se3ToVec((1/time_step)*mr.MatrixLog6(np.dot(mr.TransInv(Xd),Xd_next)))
	print('Vd is: ')
	print(Vd.shape)
	ADxxd = mr.Adjoint(np.dot(mr.TransInv(X),Xd))
	print('ADXXD IS')
	print(ADxxd.shape)
	ADxxdVd = np.dot(ADxxd,Vd)
	Xerr = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(X),Xd)))
	print('Xerr is: ')
	print(Xerr.shape)
	V = ADxxdVd + Kp*Xerr + 0
	
	print("V is: ")
	print(V.shape)
	Ja = mr.JacobianBody(Blist, thetalist)
	Jb = mr.Adjoint(np.dot(mr.TransInv(T0e),mr.TransInv(Tb0))).dot(F)
	Je = np.concatenate((Jb,Ja),axis=1)
	Je_inv = np.linalg.pinv(Je)
	print('Je is: ')
	print(Je.shape)
	command = Je_inv.dot(V)
	print("command is: ")
	print(command.shape)

# example input:

Xd = np.array([[ 0, 0, 1, 0.5],
			   [ 0, 1, 0,   0],
			   [-1, 0, 0, 0.5],
			   [ 0, 0, 0,   1]])

Xd_next = np.array([[0, 0, 1,   0.6],
					[0, 1, 0,     0],
					[-1, 0, 1, 0.3],
					[0, 0, 0,     1]])

X = np.array([[ 0.170, 0, 0.985, 0.387],
			[0, 1, 0,   0],
			[-0.985, 0, 0.170, 0.570],
			[ 0, 0, 0,     1]])
time_step = 0.01
Kp = 1
Ki = 0
# output
V = FeedbackControl(X,Xd,Xd_next,Kp,Ki,time_step)
