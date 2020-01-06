import modern_robotics as mr
import numpy as np

# This is the code for Milestone2, it computes a trajectory as a list of N SE(3) matrices 
# corresponding to the screw motion about a space screw axis


def matrix2list(whole_traj,traj,N,gripper_state):
	'''
	This function convert the transformation matrix to a N * 13 entries list.
	'''
	sub = np.zeros((N,13),dtype = float)
	for i in range(N):
		sub[i][0] = traj[i][0][0]
		sub[i][1] = traj[i][0][1]
		sub[i][2] = traj[i][0][2]
		sub[i][3] = traj[i][1][0]
		sub[i][4] = traj[i][1][1]
		sub[i][5] = traj[i][1][2]
		sub[i][6] = traj[i][2][0]
		sub[i][7] = traj[i][2][1]
		sub[i][8] = traj[i][2][2]
		sub[i][9] = traj[i][0][3]
		sub[i][10] = traj[i][1][3]
		sub[i][11] = traj[i][2][3]
		sub[i][12] = gripper_state
		whole_traj.append(sub[i].tolist())
	
	return whole_traj

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
		The discretized trajectory as an N *13 matrices. The first 12 entries are top three rows of the transformation matrix 
		Tse at that instant of time in SE(3), the last entry is the gripper state: 0 = open, 1 = closed.
		
		Example Input:

				Tse_ini = np.array([[ 0, 0, 1,   0],
														[ 0, 1, 0,   0],
														[-1, 0, 0, 0.5],
														[ 0, 0, 0,   1]])

				Tsc_ini = np.array([[1, 0, 0,     1],
														[0, 1, 0,     0],
														[0, 0, 1, 0.025],
														[0, 0, 0,     1]])

				Tsc_fin = np.array([[ 0, 1, 0,     0],
														[-1, 0, 0,    -1],
														[ 0, 0, 1, 0.025],
														[ 0, 0, 0,     1]])

				Tce_grp = np.array([[ 0, 0, 1, 0],
														[ 0, 1, 0, 0],
														[-1, 0, 0, 0],
														[ 0, 0, 0, 1]])

				Tce_sta = np.array([[ 0, 0, 1,   0],
														[ 0, 1, 0,   0],
														[-1, 0, 0, 0.2],  # 0.2 is a random number(specify the height)
														[ 0, 0, 0,   1]])

				k = 1

		Output:
				a list of 13 entries csv file:
				r11, r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz, gripper state

			 
		"""
		# create an empty list
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
		traj3 = np.asarray(mr.CartesianTrajectory(Tse_grp, Tse_grp, Tf, N, method))
		whole_traj = matrix2list(whole_traj,traj3,N,gripper_state)

		#4th step
		N = 100
		traj4 = np.asarray(mr.CartesianTrajectory(Tse_grp, Tse_sta, Tf, N, method))
		whole_traj = matrix2list(whole_traj,traj4,N,gripper_state)

		#5th step
		N = 500
		traj5 = np.asarray(mr.CartesianTrajectory(Tse_sta, Tse_finish, Tf, N, method))
		whole_traj = matrix2list(whole_traj,traj5,N,gripper_state)

		#6th step
		N = 100
		traj6 = np.asarray(mr.CartesianTrajectory(Tse_finish, Tse_fin, Tf, N, method))
		whole_traj = matrix2list(whole_traj,traj6,N,gripper_state)

		#7th step
		N = 100
		gripper_state = 0
		traj7 = np.asarray(mr.CartesianTrajectory(Tse_fin, Tse_fin, Tf, N, method))
		whole_traj = matrix2list(whole_traj,traj7,N,gripper_state)

		#8th step
		N = 100
		traj8 = np.asarray(mr.CartesianTrajectory(Tse_fin, Tse_finish, Tf, N, method))
		whole_traj = matrix2list(whole_traj,traj8,N,gripper_state)



		return whole_traj



# example input:


Tse_ini = np.array([[ 0, 0, 1,   0],
					[ 0, 1, 0,   0],
					[ -1, 0,0, 0.5],
					[ 0, 0, 0,   1]])
Tsc_ini = np.array([[1, 0, 0,     1],
					[0, 1, 0,     0],
					[0, 0, 1, 0.025],
					[0, 0, 0,     1]])

Tsc_fin = np.array([[ 0, 1, 0,     0],
					[-1, 0, 0,    -1],
					[ 0, 0, 1, 0.025],
					[ 0, 0, 0,     1]])

Tce_grp = np.array([[ -np.sqrt(2)/2, 0, np.sqrt(2)/2, 0],
					[ 0, 1, 0, 0],
					[-np.sqrt(2)/2, 0, -np.sqrt(2)/2, 0],
					[ 0, 0, 0, 1]])

Tce_sta = np.array([[ 0, 0, 1,   0],
					[ 0, 1, 0,   0],
					[-1, 0, 0, 0.1],  # 0.1 is a random number(specify the height)
					[ 0, 0, 0,   1]])
k = 1

# get the output trajectory:

traj = GenerateTrajectory(Tse_ini, Tsc_ini, Tsc_fin, Tce_grp, Tce_sta, k)
np.savetxt('end_eff.csv', traj, delimiter=',')