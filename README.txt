					ME449 Final Project

Package Description:
-finalproject.zip
    • /code
      -full_program.py  : the full program
      -nextstate.py : milestone1
      -traj_gen.py  : milestone2
      -control.py    : milestone3
    • /results
      - /best
      - /newTask
      - /overshoot
    • README.pdf

Software Explanation:

For each of the milestones, the inputs and the outputs of each function in the corresponding python file are aligned with what on the wiki Capstone page, I’ve already put the example input in the python file and by just running the python file listed above, we can get the desired output.

For the full program, my input is not exactly the same as what it is on the wiki page. On the wiki page,   the desired inputs are:
    • the initial resting configuration of the cube object (which has a known geometry), represented by a frame attached to the center of the object 
    • the desired final resting configuration of the cube object 
    • the actual initial configuration of the youBot 
    • the reference initial configuration of the youBot (which will generally be different from the actual initial configuration, to allow you to test feedback control) 
    • optionally: gains for your feedback controller (or these gains can be hard-coded in your program) 
I was a little confused about how to choose the reference initial configuration of the youBot and what is the difference between the actual initial configuration of the youBot  and the reference initial configuration of the youBot. So I decided not to give the reference initial configuration directly, instead, I give the desired input for Milestone 2 and using the GenerateTrajectory function to generate all the reference end-effector trajectory. Then enter the loop, first calculate the control law using FeedbackControl function and take the output of that function to the NextState function to calculate the new configuration.


