'''
ikin.py

   This is initial ikin for initial movement of SSRMS Canadarm 2.

'''

import rclpy
import numpy as np

from math import pi, sin, cos, acos, atan2, sqrt, fmod, exp

# Grab the utilities
from hw5code.GeneratorNode      import GeneratorNode
from hw5code.TransformHelpers   import *
from hw5code.TrajectoryUtils    import *

# Grab the general fkin from HW6 P1.
from hw6code.KinematicChain     import KinematicChain

# Import the format for the condition number message
from std_msgs.msg import Float64


#
#   Trajectory Class
#
class Trajectory():
    # Initialization.
    def __init__(self, node):
        # Set up the kinematic chain object.
        self.chain = KinematicChain(node, 'world', 'link_ee', self.jointnames())
        
        # Define the various points.
        self.njoints = 7
        self.q0 = np.radians(np.zeros(self.njoints))
        self.p0 = np.array([17.688, 1.6134, 0.0])
        self.R0 = Reye()

        self.pleft = np.array([5.127, 1.3999, 15.698])
        self.pright  = np.array([0.63958, 0.19806, -0.4366])

        self.Rleft = Reye()
        self.Rright = Reye()

        # Initialize the current/starting joint position.
        self.qd = self.q0
        self.pd = self.p0
        self.Rd = self.R0
        self.lam = 20
        
        # Setup the condition number publisher
        self.pub = node.create_publisher(Float64, '/condition', 10)


    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        return ['Shoulder_Roll', 'Shoulder_Yaw', 'Shoulder_Pitch', 'Elbow_Pitch', 'Wrist_Pitch', 'Wrist_Yaw', 'Wrist_Roll']

    # Evaluate at the given time.  This was last called (dt) ago.
    def evaluate(self, t, dt):
        
        # Decide which phase we are in:
        if t < 3.0:
            # Approach movement:
            (s0, s0dot) = goto(t, 3.0, 0.0, 1.0)

            pd = self.p0 + (self.pright - self.p0) * s0
            vd =           (self.pright - self.p0) * s0dot

            Rd = Reye()
            wd = np.zeros(3)
            
        else: 
            t = (t-3) % 10
            if t < 5.0:
                # Approach movement:
                (s0, s0dot) = goto(t, 5.0, 0.0, 1.0)

                pd = self.pright + (self.pleft - self.pright) * s0
                vd =           (self.pleft - self.pright) * s0dot
            
            else:
                (s0, s0dot) = goto(t-5, 5.0, 0.0, 1.0)

                pd = self.pleft + (self.pright - self.pleft) * s0
                vd =           (self.pright - self.pleft) * s0dot

                Rd = Reye()
                wd = np.zeros(3)

            Rd = Reye()
            wd = np.zeros(3)

        # Grab the last joint value and desired orientation.
        qdlast = self.qd
        pdlast = self.pd
        Rdlast = self.Rd

        # Compute the inverse kinematics
        (p, R, Jv, Jw) = self.chain.fkin(qdlast)
        
        vr = vd+self.lam*ep(pdlast, p)
        wr = wd+self.lam*eR(Rdlast, R)
        
        xr_dot = np.concatenate((vr, wr))
        J = np.vstack((Jv, Jw))
        
        qddot = np.linalg.pinv(J)@xr_dot

        # Integrate the joint position.
        qd = qdlast + dt * qddot

        # Save the joint value and desired orientation for next cycle.
        self.pd = pd
        self.qd = qd
        self.Rd = Rd

        # Return the desired joint and task (position/orientation) pos/vel.
        return (qd, qddot, pd, vd, Rd, wd)


#
#  Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Initialize the generator node for 100Hz udpates, using the above
    # Trajectory class.
    generator = GeneratorNode('generator', 100, Trajectory)

    # Spin, meaning keep running (taking care of the timer callbacks
    # and message passing), until interrupted or the trajectory ends.
    generator.spin()

    # Shutdown the node and ROS.
    generator.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
