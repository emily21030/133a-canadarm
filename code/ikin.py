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
        self.chain = KinematicChain(node, 'world', 'tip', self.jointnames())
        
        # Define the various points.
        self.njoints = 9
        self.q0 = np.radians(np.zeros(self.njoints))
        self.p0 = np.array([0.0, 0.55, 1.0])
        self.R0 = Reye()

        self.pleft  = np.array([0.3, 0.5, 0.15])
        self.pright = np.array([-0.3, 0.5, 0.15])

        self.Rleft = np.array([[0, 0, -1], [1, 0, 0], [0, -1, 0]])
        self.Rright = Reye()

        # Initialize the current/starting joint position.
        self.qd = self.q0
        self.pd = self.p0
        self.Rd = self.R0
        self.lam = 50
        self.L = 0.4
        
        # Setup the condition number publisher
        self.pub = node.create_publisher(Float64, '/condition', 10)


    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        return ['theta1', 'theta2', 'theta3', 'theta4', 'theta5', 'theta6']

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
            s = cos(np.pi/2.5 * (t-3))
            sdot = -pi/2.5 * np.sin(np.pi/2.5 * (t-3))
            
            pd = np.array([-0.3*s, 0.5, 0.5-0.35*s**2])
            vd = np.array([-0.3*sdot, 0.0, -0.70*s*sdot])
            
            alpha = -pi/3 * (s-1)
            alphadot = -pi/3 * sdot
            
            nleft = np.array([1, 1, -1]) / sqrt(3)
            Rd = Rotn(nleft, -alpha)
            wd = -nleft * alphadot
        

        # FIXME: REUSE THE PREVIOUS INVERSE KINEMATICS.


        # Grab the last joint value and desired orientation.
        qdlast = self.qd
        pdlast = self.pd
        Rdlast = self.Rd

        # Compute the inverse kinematics
        (p, R, Jv, Jw) = self.chain.fkin(qdlast)
        
        vr = vd+self.lam*ep(pdlast, p)
        wr = wd+self.lam*eR(Rdlast, R)
        
        vec6 = np.concatenate((vr, wr))
        J = np.vstack((Jv, Jw))
        
        qddot = np.linalg.inv(J)@vec6
        
        # Compute and publish the condition number
        Jbar = np.diag([1/self.L, 1/self.L, 1/self.L, 1, 1, 1]) @ J
        condition = np.linalg.cond(Jbar)
        self.pub.publish(Float64(data=condition))

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
