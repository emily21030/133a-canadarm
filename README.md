# 133a-canadarm
Team repo for ME/CS 133a project. Clone this project into your ~/robotws/src/ directory on VM.

## Current files to run!!!
RVIZ: rviz/viewmarkers.rviz

URDF: urdf/SSRMS_Canadarm2.urdf

Python: code/ikin_node.py

### Major Update Meeting Notes 2024/12/02
Feedback:
- Orientation is already specified in zero pos., now just need to specify orientation to be perpendicular to the paddle
  - Extra thought: specify normal vector (2 DOF), not all 3 DOFs of tip orientation --> 5 (= 3 pos. + 2 orientation) x7 Jacobian
- Approximate the work space as a sphere, don't need to deal with problem of solving for where the singularities are 
- Next steps:
  - Randomize asteroid trajectory
  - Secondary task chicken dance elbow to avoid asteroid
- Final report: explain trade-offs and reasonings for decision/design in our robot

### SOS OH Meeting Notes 2024/12/02
Notes:
- Unable to setup/build ROS with launch in setup.py --> how to create launch files?

Feedback:
- Random asteroid trajectory:
  - Pick intersection point and randomize angle of opening (over [5º, 90º], [-5º, -90º]) from line between starting point and origin (middle of robot)
  - OR pick random starting position/velocity of asteroid, then compute intersection of asteroid with work space
  - OR pick random starting position of asteroid, randomize inner circle radius (over defined min/max radius) and follow path of tangential chord from intersection point to inner circle
      - Solve for intersection point of asteroid with work space in this case
- Catching movement:
  - goto() to go to intersection/catching point
  - spline(v0=v, vf=0) to decelerate astseroid

### Preliminary Update Meeting Notes 2024/11/25
Notes:
- Able to view .urdf and have some movement with the Canadarm
- How to render asteroids? With Gazebo (https://gazebosim.org/docs/latest/ros2_integration/)?
- Next step is we want to generate asteroids travelling at random velocities, and to compute the ikin for the arm to hit the asteroid. How do we break this problem down into smaller steps?

Feedback:
- Rendering asteroids: save CAD file into ROS2 package, tell rviz to render CAD file at a particular location --> keep it simple and make it a sphere
- For positioning, functionally we have 4 DOFs (since 3 wrist DOFs don't really affect it) so only 1 DOF for secondary task
- Secondary task: use elbow DOF to 'repulse' and avoid asteroid to not hit it while catching
- Far ahead: put the base on "thrusters" to make it a 10 DOF system
- Gazebo is for forces, physics engine and rendering, we should not use it M1 cannot run it

Tasks:

Emily:
- Use weighted Jacobian to handle ikin near singularities
- End effector: Spherical velcro ball or paddle with velcro (to introduce orientation)
  
Emily and Kim:
- Implement as mechanism to 'turn on/off electromagnet' where 'on' tracks asteroid location with robot tip
  - Create deceleration of asteroid by saying velocity should slow down while arm is in contact with asteroid
  - Spline: slow, speed up to match speed of asteroid at location, and slow down to stopped
  - Pick meeting location that is at earliest point of intersection in workspace and slow to stop at last point of intersection (maximize contact with asteroid) --> render the asteroid outside of the workspace at first
    - ^ Solve the singularity problem

Jimmy:
- Render asteroid and randomize movement
  - Send this separate from .urdf --> look at ball demo, and grab useful pieces into our code
 


  

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
