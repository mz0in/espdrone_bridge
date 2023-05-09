#! /usr/bin/python3

"""
Command the esp-drone to a disired position
"""
import logging
import time
import numpy as np
from threading import Thread
from scipy.spatial.transform import Rotation

import tf

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.utils.callbacks import Caller

import rospy
from geometry_msgs.msg import PoseStamped, Transform, Twist, Vector3, Quaternion
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint, JointTrajectory, JointTrajectoryPoint
# from mav_msgs.msg import RollPitchYawrateThrust
from mavros_msgs.msg import AttitudeTarget

logging.basicConfig(level=logging.ERROR)


# class LeeController():
#     """LeeController
#     """

#     def __init__(self):
#         # lee parameters
#         self.position_gain_ = [6, 6, 6]
#         self.velocity_gain_ = [4.7, 4.7, 4.7]
#         self.attitude_gain_ = [3, 3, 0.035]
#         self.angular_rate_gain_ = [0.52, 0.52, 0.025]
#         self.command_trajectory_ = MultiDOFJointTrajectoryPoint()
#         self.odom = PoseStamped()
#         self.normalized_attitude_gain_ = np.diag(self.attitude_gain_)
#         self.normalized_angular_rate_gain_ = np.diag(self.angular_rate_gain_)
#         self.I = np.diag([1e-6, 1e-6, 1e-6, 1])
#         self.mass = 0.05
#         self.velocity = np.zeros((3, 1))
#         self.acceleration_W = np.zeros((3, 1))
#         self.g = np.array([0.,0.,9.8])


#     def CalculateRotorVelocities(self):
#         rotor_velocities = []
#         return rotor_velocities

#     def CalculateRPYT(self):
#         rpyt = np.zeros((4, 1))
#         acc = self.ComputeDesiredAcceleration()
#         angular_acc = self.ComputeDesiredAngularAcc(acc)
#         R = Rotation.from_quat(self.odom.pose.orientation)
#         thrust = -self.mass * (acc.T * R.as_matrix[:, 2])
#         rpyt[:3] = angular_acc
#         rpyt[3] = thrust
#         return rpyt

#     def SetOdometry(self, odom):
#         self.odom = odom

#     def SetTrajectoryPoint(self, trajectory_point):
#         self.command_trajectory_ = trajectory_point
#         self.command_trajectory_.time_from_start = rospy.Duration(1.0)
#         self.command_trajectory_.transforms.append(Transform(Vector3(1.0, 2.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)))
#         self.command_trajectory_.velocities.append(Twist(Vector3(1.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)))

#     def ComputeDesiredAcceleration(self):
#         acceleration = np.zeros((3, 1))
#         position_error = self.odom.pose.position - self.command_trajectory_.transforms[0].translation
#         R_W_I = Rotation.from_quat(self.odom.pose.orientation).as_matrix()
#         velocity_W = R_W_I * self.velocity
#         velocity_error = velocity_W - self.command_trajectory_.velocities[0]
#         e_3 = np.array([0, 0, 1])
#         acceleration = ((position_error * self.position_gain_)+velocity_error*self.velocity_gain_)/self.mass - self.g * e_3 - self.acceleration_W
#         return acceleration

#     def ComputeDesiredAngularAcc(self, acceleration):
#         angular_acc = np.zeros((3, 1))
#         R = Rotation.from_quat(self.odom.pose.orientation).as_matrix()
#         yaw = Rotation.from_quat(self.command_trajectory_.transforms[0].rotation).as_euler('xyz')[2]
#         b1_des = np.array([np.cos(yaw), np.sin(yaw), 0])
#         b3_des = acceleration / np.linalg.norm(acceleration)
#         b2_des = np.cross(b3_des, b1_des)
#         b2_des = b2_des / np.linalg.norm(b2_des)
#         R_des = np.array([b1_des, b2_des, b3_des]).T
#         angle_error = 0.5 * (np.trace(R_des.T * R) - 1)
#         angle_error = np.clip(angle_error, -1, 1)
#         angle_error = np.diag(angle_error)
#         angular_rate_des = np.array([0, 0, 0])
#         # angular_rate_des = self.command_trajectory_
#         angular_rate_error = self.angular_velocity - angular_rate_des
#         angular_acc = -1 * self.normalized_attitude_gain_ * angle_error - self.normalized_angular_rate_gain_ * angular_rate_error
#         return angular_acc

class ESPPlanner:
    """Example that connects to a Crazyflie and ramps the motors up/down and
    the disconnects"""

    def __init__(self, link_uri):
        """ Initialize and run the example with the specified link_uri """
        rospy.init_node("aerial_catcher_esp")
        
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback, queue_size=10)

        rospy.Subscriber("/aerial_catcher/esp/attitude",AttitudeTarget, self.update_cmd, queue_size=1)

        self._cf = Crazyflie(rw_cache='./cache')
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)
        self._cf.open_link(link_uri)

        self.target_pose = PoseStamped()
        self.connected = True
        self.pose = PoseStamped()
        self.rate = rospy.Rate(50)
        print('Connecting to %s' % link_uri)
        self._cf.commander.send_setpoint(0, 0, 0, 0)
        # self._cf.commander.send_position_setpoint(0, 0, 0, 0)
        # for lee controller
        # lee_controller = LeeController()

    def pose_callback(self, pose:PoseStamped):
        self.pose = pose
        x = pose.pose.position.x
        y = pose.pose.position.y
        z = pose.pose.position.z
        self._cf.loc.send_extpos([x, y, z])
        # print("self.pose", self.pose)

    def update_cmd(self, cmd:AttitudeTarget):
        # print("cmd", cmd)
        quat = cmd.orientation
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w
        ang = Rotation.from_quat((x, y, z, w)).as_euler('xyz')
        cmd.thrust = int(cmd.thrust)
        if cmd.thrust > 59999:
            cmd.thrust = 59999
        if cmd.thrust < 10002:
            cmd.thrust = 10002
        # cmd.thrust = 20000
        self._cf.commander.send_setpoint(ang[0], ang[1], ang[2], cmd.thrust)
        # print(cmd.thrust)

    def main(self):
        self.target_pose.header.stamp = rospy.Time.now()
        self.target_pose.pose.position.x = 0
        self.target_pose.pose.position.y = 0
        self.target_pose.pose.position.z = 1
        self.target_pose.pose.orientation.x = 0
        self.target_pose.pose.orientation.y = 0
        self.target_pose.pose.orientation.z = 0
        self.target_pose.pose.orientation.w = 1
        while not rospy.is_shutdown():
            # self.target_pub.publish(self.target_pose)
            self.rate.sleep()

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print("connected!")
        # Start a separate thread to do the motor test.
        # Do not hijack the calling thread!
        # Thread(target=self._ramp_motors).start()

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))
        self.connected = False

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
        self.connected = False

    def _ramp_motors(self):
        thrust_mult = 1
        thrust_step = 500
        thrust = 0xFFFF-1
        pitch = 0
        roll = 0
        yawrate = 0

        # Unlock startup thrust protection
        self._cf.commander.send_setpoint(0, 0, 0, 0)

        while thrust >= 20000:
            self._cf.commander.send_setpoint(roll, pitch, yawrate, thrust)
            time.sleep(0.1)
            if thrust >= 25000:
                thrust_mult = -1
            thrust += thrust_step * thrust_mult
        self._cf.commander.send_setpoint(0, 0, 0, 0)
        # Make sure that the last packet leaves before the link is closed
        # since the message queue is not flushed before closing
        time.sleep(0.1)
        self._cf.close_link()

    def __del__(self):
        self._cf.commander.send_setpoint(0, 0, 0, 0)


if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    le1 = ESPPlanner('udp://192.168.43.42')
    while (le1.connected):
        # le1._ramp_motors()
        time.sleep(1)
