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

# Ball demo imports

from rclpy.node                 import Node
from rclpy.qos                  import QoSProfile, DurabilityPolicy
from rclpy.time                 import Duration
from geometry_msgs.msg          import Point, Vector3, Quaternion
from std_msgs.msg               import ColorRGBA
from visualization_msgs.msg     import Marker
from visualization_msgs.msg     import MarkerArray




#
#   Demo Node Class
#
class DemoNode(Node):
    # Initialization.
    def __init__(self, name, rate):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # Prepare the publisher (latching for new subscribers).
        quality = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)
        self.pub = self.create_publisher(
            MarkerArray, '/visualization_marker_array', quality)

        # Initialize the ball position, velocity, set the acceleration.
        self.radius = 0.1

        self.p = np.array([0.0, 0.0, self.radius])
        self.v = np.array([1.0, 0.1,  5.0       ])
        self.a = np.array([0.0, 0.0, -9.81      ])

        # Create the sphere marker.
        diam        = 2 * self.radius
        self.marker = Marker()
        self.marker.header.frame_id  = "world"
        self.marker.header.stamp     = self.get_clock().now().to_msg()
        self.marker.action           = Marker.ADD
        self.marker.ns               = "point"
        self.marker.id               = 1
        self.marker.type             = Marker.SPHERE
        self.marker.pose.orientation = Quaternion()
        self.marker.pose.position    = Point_from_p(self.p)
        self.marker.scale            = Vector3(x = diam, y = diam, z = diam)
        self.marker.color            = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
        # a = 0.8 is slightly transparent!

        # Create the marker array message.
        self.markerarray = MarkerArray(markers = [self.marker])

        # Set up the timing so (t=0) will occur in the first update
        # cycle (dt) from now.
        self.dt    = 1.0 / float(rate)
        self.t     = -self.dt
        self.start = self.get_clock().now() + Duration(seconds=self.dt)

        # Create a timer to keep calling update().
        self.create_timer(self.dt, self.update)
        self.get_logger().info("Running with dt of %f seconds (%fHz)" %
                               (self.dt, rate))

    # Shutdown
    def shutdown(self):
        # Destroy the node, including cleaning up the timer.
        self.destroy_node()

    # Return the current time (in ROS format).
    def now(self):
        return self.start + Duration(seconds=self.t)

    # Update - send a new joint command every time step.
    def update(self):
        # To avoid any time jitter enforce a constant time step and
        # integrate to get the current time.
        self.t += self.dt

        # Integrate the velocity, then the position.
        self.v += self.dt * self.a
        self.p += self.dt * self.v

        # Check for a bounce - not the change in x velocity is non-physical.
        if self.p[2] < self.radius:
            self.p[2] = self.radius + (self.radius - self.p[2])
            self.v[2] *= -1.0
            self.v[0] *= -1.0   # Change x just for the fun of it!

        # Update the ID number to create a new ball and leave the
        # previous balls where they are.
        #####################
        self.marker.id += 1
        #####################

        # Update the message and publish.
        self.marker.header.stamp  = self.now().to_msg()
        self.marker.pose.position = Point_from_p(self.p)
        self.pub.publish(self.markerarray)


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
        gamma = 0.5
        J_Winv = np.linalg.inv(np.transpose(J)@J + gamma**2*np.eye(7))@np.transpose(J)
        
        qddot = J_Winv@xr_dot

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
    # Initialize ROS and the demo node (100Hz).
    rclpy.init(args=args)
    #node = DemoNode('balldemo', 100)

    # Initialize the generator node for 100Hz udpates, using the above
    # Trajectory class.
    generator = GeneratorNode('generator', 100, Trajectory)

    # Run until interrupted.
    #rclpy.spin(node)

    # Spin, meaning keep running (taking care of the timer callbacks
    # and message passing), until interrupted or the trajectory ends.
    generator.spin()

    # Shutdown the node and ROS.
    #node.shutdown()
    generator.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
