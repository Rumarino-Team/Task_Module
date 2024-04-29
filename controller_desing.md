
Controller Documentation:

Date for the pre-qualifiers

Minimum Product tasks for the Controllers. The following is the design document for the Controller REPO. The aim is to build a custom simpler controller of Hydrus.


Previous work and why we choose to make this repository.

We choose to create this repository because of the previous problems with the uuv_controller simulation. Specially when trying to integrate the controllers on new vehicles. We initially used that repository, because it enable with it a simulation platform but when trying to integrate simulations, but after trying to integrate our new vehicles URDF and fail, and trying to optimize the the controllers parameters and fail. we decided that we needed to hack this part with a simpler and more flexible approach.





Control System:
Model -> Camera -> Task_Module -> Controller ->Thrusters

The idea of the custom controller will be to limit the number of free degrees the submarine have. Instead of tuning a PID controller with 6 DOF, we will focus on simpler tasks like  yaw (rotation) Surge (moving forward and backwards) and Swaying (moving up and down).


The reason for this desicion is because tecnically speaking adopting a 6DOF (6 degree of freedom) controller is more complex to tune and we dont know how well is going to work on the robot without a good simulation for doing controllers tuning. While with a simpler 3DOF (3 degree of freedom) we can focus on single tasks like moving forward, rotate and dive without worrying about roll, pitch and other movements that are not critical for our applications.


The plan could also be to implement PID separate controllers assiggning each controller a set of different thrusters. The starting idea will be to assinging the lateral trhusters (4) and assing them to the same Trhuster Allocation Matrix (TAM). And assign the central thrusters (4) to a diferent TAM. This way we can tune each controller independently without each one interefering to the others. We made this separation becuase we made the assuption that the central trhusters do not have any effect on the yaw axis , and the Surge axis and viceversa the lateral trhusters dont affect the yaw rotation axis.

Implementation idea:

1. Define PID controllers for each task:

2. The definition of the TAM matrix will be obviate at the start. Because we are building it on the prior beliefs that lateral thrusters only affect surge and sway, central thrusters only affect yaw and most importantly that for the degree of fredom we are working with all the trhusters have the same contribution each time. Meaning thaat when using the trhusters they will all act at the same time to execute the desired movement. In a future development we could extend this functioanlity to allow different trhuster allocation for different vehicles.

3. The path planning will be execute in the Task_module. In fact the path planning will be straight forward in the sense that we are only going to allow  limited types of movements. For example if we need to go down and the fornt left. We will first execute the command for going down by sending the desired depth to the depth controller. Once that movement is completed we will send the command for moving forward to the surge controller.

4. In order to optimize this we will execute an application for tuning the PID parameters online. The application will allow sending step inputs to each controller and monitor the output and calculate the error signal.

5. With this information we will be able to tune the PID gains Kp, Ki and Kd for fast responses and minimize oscillations.

