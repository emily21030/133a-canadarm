# 133a-canadarm
Team repo for ME/CS 133a project. Clone this project into your ~/robotws/src/ directory on VM.

### Preliminary Update Meeting Notes 2024/11/25
Notes:
- Able to view .urdf and have some movement with the Canadarm
- How to render asteroids? With Gazebo (https://gazebosim.org/docs/latest/ros2_integration/)?
- Next step is we want to generate asteroids travelling at random velocities, and to compute the ikin for the arm to hit the asteroid. How do we break this problem down into smaller steps?


### Initial Meeting Notes 2024/11/19
Robot: Canadarm
Task: Catch a rogue floating astronaut? Throw the ISS?

Notes:
- 3D printing file provided by CSA (https://www.asc-csa.gc.ca/eng/multimedia/search/image/10186) 
- .urdf file made by independent user (https://github.com/vyas-shubham/TraceableRobotModels/blob/master/SRMS_Canadarm/SRMS_Canadarm.urdf) 

Feedback:
- Slow asteroid catching problem, asteroid deflection
- Minimize cost function of different arm orientations
- Handling near-singularity behaviours
- Action Item: Canadarm hit moving asteroid 
