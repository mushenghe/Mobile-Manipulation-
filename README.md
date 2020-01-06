# Mobile Manipulation
ME-449 Capstone Project: YouBot KUKA Manipulation. 

## Project Description:

This is the final project of course [ME-449 Robotic Manipulation](http://hades.mech.northwestern.edu/index.php/ME_449_Robotic_Manipulation) at Northwestern University. This is also the capstone Project for the Coursera "Modern Robotics" specialization. The detailed project description can be find [here](http://hades.mech.northwestern.edu/index.php/Mobile_Manipulation_Capstone#Milestone_1:_youBot_Kinematics_Simulator_and_csv_Output)

The goal of this project is to drive the KUKA youBot to pick up a block at the start location, carry it to the desired location, and put it down in the simulation software V-REP. The project covers the following topics:<br>
1. Plan a trajectory for the end-effector of the youBot mobile manipulator<br>
2. Generate the kinematics model of the youBot, consisting of the mobile base with 4 mecanum wheels and the robot arm with 5 joints<br>
3. Apply feedback control to drive the robot to implement the desired task<br>
4. Conduct the simulations in V-REP

## Package Description:
##### finalproject.zip<br>
   • /code<br>
        -full_program.py : the full program<br>
        -nextstate.py : milestone1<br>
        -traj_gen.py : milestone2<br>
        -control.py : milestone3<br>
   • /results<br>
        - /best<br>
        - /newTask<br>
        - /overshoot<br>
## Result:

1. when the initial configuration for the cube is:<br>
Tsc_ini = <br>
[1, 0, 0,     1],<br>
[0, 1, 0,     0],<br>
[0, 0, 1, 0.025],<br>
[0, 0, 0,     1]])<br>
The goal configuration for the cube is:<br>
Tsc_fin =<br>
[ 0, 1, 0,     0],<br>
[-1, 0, 0,    -1],<br>
[ 0, 0, 1, 0.025],<br>
[ 0, 0, 0,     1]])<br>
![](/results/overshoot/overshoot.gif)
2. when the initial configuration for the cube is:<br>
Tsc_ini = <br>
[1, 0, 0,     1],<br>
[0, 1, 0,     0.5],<br>
[0, 0, 1, 0.025],<br>
[0, 0, 0,     1]])<br>
The goal configuration for the cube is:<br>
Tsc_fin =<br>
[ 0, 1, 0,     0.2],<br>
[-1, 0, 0,    -2],<br>
[ 0, 0, 1, 0.025],<br>
[ 0, 0, 0,     1]])<br>
![](/results/newTask/newtask.gif)

The corresponding six elements of end-effector error twist along with time are:
1. KP = 1.5, KI = 0<br>
![](/results/best/BEST.png)
2. KP = 30, KI = 10<br>
![](/results/newTask/newTask.png)


