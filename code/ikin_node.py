'''ikin_node.py

   This is a demo for Canadarm2

   Node:      /catch
   Publish:   /joint_states             sensor_msgs.msg.JointState
   Broadcast: 'pelvis' w.r.t. 'world'   geometry_msgs.msg.TransformStamped

'''

import rclpy
import numpy as np

from math import pi, sin, cos, acos, atan2, sqrt, fmod, exp

from rclpy.node                 import Node
from rclpy.time                 import Duration
from tf2_ros                    import TransformBroadcaster
from geometry_msgs.msg          import TransformStamped
from sensor_msgs.msg            import JointState

from code.TransformHelpers     import *
from hw5code.TrajectoryUtils    import *
from hw6code.KinematicChain     import KinematicChain

# Marker liraries
from rclpy.qos                  import QoSProfile, DurabilityPolicy
from geometry_msgs.msg          import Point, Vector3, Quaternion
from std_msgs.msg               import ColorRGBA
from visualization_msgs.msg     import Marker
from visualization_msgs.msg     import MarkerArray


#
#   Canadarm2 Joint Names
#
jointnames = ['Shoulder_Roll', 'Shoulder_Yaw', 'Shoulder_Pitch', 'Elbow_Pitch', 'Wrist_Pitch', 'Wrist_Yaw', 'Wrist_Roll']


#
#   Demo Node Class
#
class DemoNode(Node):
    # Initialization.
    def __init__(self, name, rate):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # Initialize the transform broadcaster
        self.broadcaster = TransformBroadcaster(self)

        # Add a publisher to send the joint commands.
        self.jpub = self.create_publisher(JointState, '/joint_states', 10)

        # Prepare the publisher (latching for new subscribers).
        quality = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)
        self.mpub = self.create_publisher(
            MarkerArray, '/visualization_marker_array', quality)

        # Wait for a connection to happen.  This isn't necessary, but
        # means we don't start until the rest of the system is ready.
        self.get_logger().info("Waiting for a /joint_states subscriber...")
        while(not self.count_subscribers('/joint_states')):
            pass

        # Set up the timing so (t=0) will occur in the first update
        # cycle (dt) from now.
        self.dt    = 1.0 / float(rate)
        self.t     = -self.dt
        self.start = self.get_clock().now() + Duration(seconds=self.dt)

        # Initialize ikin variables
        # Set up the kinematic chain object.
        self.chain = KinematicChain(self, 'world', 'link_ee', jointnames)
        
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

        # Initialize the ball position, angle parameters, workspace parameters, velocity, set the acceleration.
        self.r_spawn = 25
        self.ws_i = 5
        self.ws_o = 15
        self.theta_i = np.arcsin(self.ws_i/self.r_spawn)
        self.theta_o = np.arcsin(self.ws_o/self.r_spawn)
        self.theta_m = (self.theta_i + self. theta_o)/2

        self.radius = 1.0
        self.speed = 2

        #generate random unit vector for ball spawn
        v = np.random.rand(3) - 0.5
        mag = np.linalg.norm(v)
        v_hat = v/mag

        self.p = self.r_spawn * v_hat

        # Generate random angle to add
        rand = np.random.uniform(self.theta_i, self.theta_m)
        p_m = [-1, 1]
        mult = np.random.choice(p_m)
        rand_angle = rand * mult
        rotation = Rotx(rand_angle)

        a_to_o = -self.p/np.linalg.norm(self.p)
        direction = a_to_o @ rotation

        # set ball velocity 
        self.v = self.speed*direction
        self.a = np.array([0.0, 0.0, 0.0      ])
        
        # define starting and ending points for the ball
        self.pstart = self.p
        self.pend = self.p + self.v*15 # np.array([9.5, -2.5, 1.5])

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

        # Create a timer to keep calling update().
        self.create_timer(self.dt, self.update)
        self.get_logger().info("Running with dt of %f seconds (%fHz)" %
                               (self.dt, rate))
                               
        self.caught = False
        

    # Shutdown.
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

        t = self.t
        dt = self.dt

        # For the ball, Integrate the velocity, then the position.
        self.v += self.dt * self.a
        self.p += self.dt * self.v

        # Check for paddle
        # Check for a bounce - not the change in x velocity is non-physical.
        # if self.p[2] < self.radius:
        #     self.p[2] = self.radius + (self.radius - self.p[2])
        #     self.v[2] *= -1.0
        #     self.v[0] *= -1.0   # Change x just for the fun of it!

        # Update the ID number to create a new ball and leave the
        # previous balls where they are.
        #####################
        # self.marker.id += 1
        #####################

        # Decide which phase we are in for the arm:
        if t < 3.0:
            # Approach movement:
            
            (s0, s0dot) = goto(t, 3.0, 0.0, 1.0)

            pd = self.p0 + (self.pstart - self.p0) * s0
            vd =           (self.pstart - self.p0) * s0dot

            Rd = Reye()
            wd = np.zeros(3)
            
        elif t < 15.0: 
            t = t-3
            # Final movement:
            (s0, s0dot) = spline(t, 12.0, 0.0, 1.0, 0.0, 0.0)
            
            pd = self.pstart + (self.pend - self.pstart) * s0
            vd =           (self.pend - self.pstart) * s0dot

            Rd = Reye()
            wd = np.zeros(3)
            
            # attach ball to arm trajectory
            self.p = pd
            self.v = vd
        else:
            
            #generate new ball
            v = np.random.rand(3) - 0.5
            mag = np.linalg.norm(v)
            v_hat = v/mag

            self.p = self.r_spawn * v_hat
            # Generate random angle
            rand = np.random.uniform(self.theta_i, self.theta_m)
            p_m = [-1, 1]
            mult = np.random.choice(p_m)
            rand_angle = rand * mult
            rotation = Rotx(rand_angle)

            a_to_o = -self.p/np.linalg.norm(self.p)
            direction = a_to_o @ rotation

            self.v = self.speed*direction

            # set update values (or else error)
            (s0, s0dot) = goto(t, 3.0, 0.0, 1.0)

            pd = self.p0 + (self.pstart - self.p0) * s0
            vd =           (self.pstart - self.p0) * s0dot

            Rd = Reye()
            wd = np.zeros(3)

            #reset timer
            self.t = 0

            # return None

            
        # Update the message and publish.
        self.marker.header.stamp  = self.now().to_msg()
        self.marker.pose.position = Point_from_p(self.p)
        self.mpub.publish(self.markerarray)

        # Grab the last joint value and desired orientation.
        qdlast = self.qd
        pdlast = self.pd
        Rdlast = self.Rd

        # Compute the inverse kinematics
        (p, R, Jv, Jw) = self.chain.fkin(qdlast)
        
        vr = vd+self.lam*ep(pdlast, p)
        wr = wd+self.lam*eR(Rdlast, R)
        
        # convert to tip frame
        n_isol = np.array([[1, 0, 0], [0, 1, 0]])
        Jn_tip = np.transpose(R)@Jw
        
        p_norm = np.array([0, 0, 1])
        n = R@p_norm
        nd = (-1)*self.v
        en = cross(n, nd)
        nrdot_tip = np.transpose(R)@nd+self.lam*en

        
        xr_dot = np.concatenate((vr, nrdot_tip))
        J = np.vstack((Jv, Jn_tip))
        gamma = 0.7
        J_Winv = np.linalg.inv(np.transpose(J)@J + gamma**2*np.eye(7))@np.transpose(J)
        
        qddot = J_Winv@xr_dot

        # Integrate the joint position.
        qd = qdlast + dt * qddot

        # Save the joint value and desired orientation for next cycle.
        self.pd = pd
        self.qd = qd
        self.Rd = Rd

        # Build up a command message and publish.
        cmdmsg = JointState()
        cmdmsg.header.stamp = self.now().to_msg()       # Current time for ROS
        cmdmsg.name         = jointnames                # List of names
        cmdmsg.position     = qd.flatten().tolist()      # List of positions
        cmdmsg.velocity     = qddot.flatten().tolist()   # List of velocities
        self.jpub.publish(cmdmsg)



#
#  Main Code
#
def main(args=None):
    # Initialize ROS and the demo node (100Hz).
    rclpy.init(args=args)
    node = DemoNode('catch', 100)

    # Spin, until interrupted.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
