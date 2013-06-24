#!/usr/bin/env python
#
# adapted from turtlebot_node code, <dominik@dobots.nl>
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id: __init__.py 11217 2010-09-23 21:08:11Z kwc $

import roslib; roslib.load_manifest('robo50_node')

"""
ROS Robo50 node.
This driver is based on turtlebot_node (turtlebot-ros-pkg) and
turtlebot_driver.
"""

import os
import sys
import select
import serial
import termios
import time

from math import sin, cos

import roslib.rosenv
import rospy
import tf

from geometry_msgs.msg import Point, Pose, Pose2D, PoseWithCovariance, \
    Quaternion, Twist, TwistWithCovariance, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

from robo50_node.robo50_sensor_handler import Robo50SensorHandler
from robo50_node.srv import SetMotor, SetMotorResponse
from robo50_node.diagnostics import Robo50Diagnostics
from robo50_node.msg import Robo50SensorState
from robo50_node.driver import Robo50, WHEEL_SEPARATION, MAX_WHEEL_SPEED
from robo50_node.covariances import \
     ODOM_POSE_COVARIANCE, ODOM_POSE_COVARIANCE2, ODOM_TWIST_COVARIANCE, ODOM_TWIST_COVARIANCE2

#dynamic reconfigure
import dynamic_reconfigure.server
from robo50_node.cfg import Robo50Config


class Robo50Node(object):

    _SENSOR_READ_RETRY_COUNT = 5 

    def __init__(self, default_port='/dev/ttyACM0', default_update_rate=30.0):

        """
        @param default_port: default tty port to use for establishing
            connection to Robo50.  This will be overriden by ~port ROS
            param if available.
        """
        self.default_port = default_port
        self.default_update_rate = default_update_rate

        self.robot = Robo50()
        self.sensor_handler = None
        self.sensor_state = Robo50SensorState()
        self.req_cmd_vel = None
        self.drive_cmd = self.robot.direct_drive

        rospy.init_node('robo50')
        self._init_params()
        self._init_pubsub()
        
        self._pos2d = Pose2D() # 2D pose for odometry

        self._diagnostics = Robo50Diagnostics()
#         if self.imu_used:
#             self._imu = Robo50Imu()
#         else:
#             self._imu = None
            
        dynamic_reconfigure.server.Server(Robo50Config, self.reconfigure)

    def start(self):
        log_once = True
        while not rospy.is_shutdown():
            try:
                self.robot.start(self.port, self.baudrate)
                break
            except serial.serialutil.SerialException as ex:
                msg = "Failed to open port %s.  Please make sure the Create cable is plugged into the computer. \n"%(self.port)
                self._diagnostics.node_status(msg,"error")
                if log_once:
                    log_once = False
                    rospy.logerr(msg)
                else:
                    sys.stderr.write(msg)
                time.sleep(3.0)

        self.sensor_handler = Robo50SensorHandler(self.robot) 

        self.robot.control()
        # Write driver state to disk
        with open(connected_file(), 'w') as f:
            f.write("1")

#         # Startup readings from Create can be incorrect, discard first values
#         s = Robo50SensorState()
#         try:
#             self.sense(s)
#         except Exception:
#             # packet read can get interrupted, restart loop to
#             # check for exit conditions
#             pass


    def _init_params(self):
        self.port = rospy.get_param('~port', self.default_port)
        self.baudrate = rospy.get_param('~baudrate', 115200)
        #self.baudrate = rospy.get_param('~baudrate', self.default_baudrate)
        self.update_rate = rospy.get_param('~update_rate', self.default_update_rate)
        self.imu_used = rospy.get_param('~imu_used', False)
        self.odom_angular_scale_correction = rospy.get_param('~odom_angular_scale_correction', 1.0)
        self.odom_linear_scale_correction = rospy.get_param('~odom_linear_scale_correction', 1.0)
        self.cmd_vel_timeout = rospy.Duration(rospy.get_param('~cmd_vel_timeout', 0.6))
        self.stop_motors_on_bump = rospy.get_param('~stop_motors_on_bump', True)
        self.min_abs_yaw_vel = rospy.get_param('~min_abs_yaw_vel', None)
        self.max_abs_yaw_vel = rospy.get_param('~max_abs_yaw_vel', None)
        self.publish_tf = rospy.get_param('~publish_tf', False)
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.base_frame = rospy.get_param('~base_frame', 'base_footprint')
        self.move = rospy.get_param('~move', True)

        rospy.loginfo("port: %s"%(self.port))
        rospy.loginfo("baudrate: %s"%(self.baudrate))
        rospy.loginfo("update_rate: %s"%(self.update_rate))
        rospy.loginfo("imu_used: %s"%(self.imu_used))

    def _init_pubsub(self):
        self.joint_states_pub = rospy.Publisher('joint_states', JointState)
        self.odom_pub = rospy.Publisher('odom', Odometry)
        self.sensor_state_pub = rospy.Publisher('~sensor_state', Robo50SensorState)
        
        self.set_motor_srv = rospy.Service('~set_motor', SetMotor, self.set_motor)
        
        self.cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel)

        self.transform_broadcaster = None
        if self.publish_tf:
            self.transform_broadcaster = tf.TransformBroadcaster()
    
    def reconfigure(self, config, level):
        self.update_rate = config['update_rate']
        self.imu_used = config['imu_used']
        self.odom_angular_scale_correction = config['odom_angular_scale_correction']
        self.odom_linear_scale_correction = config['odom_linear_scale_correction']
        self.cmd_vel_timeout = rospy.Duration(config['cmd_vel_timeout'])
        self.stop_motors_on_bump = config['stop_motors_on_bump']
        self.min_abs_yaw_vel = config['min_abs_yaw_vel']
        self.max_abs_yaw_vel = config['max_abs_yaw_vel']
        self.move = config['move']
        return config

    def cmd_vel(self, msg):
        # Clamp to min abs yaw velocity, to avoid trying to rotate at low
        # speeds, which doesn't work well.
        if self.min_abs_yaw_vel is not None and msg.angular.z != 0.0 and abs(msg.angular.z) < self.min_abs_yaw_vel:
            msg.angular.z = self.min_abs_yaw_vel if msg.angular.z > 0.0 else -self.min_abs_yaw_vel
        # Limit maximum yaw to avoid saturating the gyro
        if self.max_abs_yaw_vel is not None and self.max_abs_yaw_vel > 0.0 and msg.angular.z != 0.0 and abs(msg.angular.z) > self.max_abs_yaw_vel: 
            msg.angular.z = self.max_abs_yaw_vel if msg.angular.z > 0.0 else -self.max_abs_yaw_vel 
        
        # convert twist to direct_drive args
        ts  = msg.linear.x * 1000 # m -> mm
        tw  = msg.angular.z  * (WHEEL_SEPARATION / 2) * 1000 
        # Prevent saturation at max wheel speed when a compound command is sent.
        if ts > 0:
            ts = min(ts,   MAX_WHEEL_SPEED - abs(tw))
        else:
            ts = max(ts, -(MAX_WHEEL_SPEED - abs(tw)))
        self.req_cmd_vel = int(ts - tw), int(ts + tw)

    def _robot_reboot(self):
        """
        Perform a robot reboot
        """
        #TODO -oDE: implement if necessary
        pass
    
    def set_motor(self, req):
        if not self.robot.serialHandler:
            raise Exception("Robot not connected")
        
        direction = 1 if req.speed > 0 else -1
        self.robot.setMotor(req.motor_id, direction, abs(req.speed))
        print "set motor %s to %s" %(req.motor_id, req.speed)
        return SetMotorResponse(True)

    def sense(self, sensor_state):
        if self.sensor_handler.getSensorData(sensor_state):
            if self.imu_used:
                self.sensor_handler.update_imu_calibration(sensor_state)
            return True
        else:
            return False
#         if self._imu:
#             self._imu.update_calibration(sensor_state)

    def spin(self):

        # state
        s = self.sensor_state
        odom = Odometry(header=rospy.Header(frame_id=self.odom_frame), child_frame_id=self.base_frame)
        js = JointState(name = ["left_wheel_joint", "right_wheel_joint", "front_castor_joint", "back_castor_joint"],
                        position=[0,0,0,0], velocity=[0,0,0,0], effort=[0,0,0,0])

        r = rospy.Rate(self.update_rate)
        last_cmd_vel = 0, 0
        last_cmd_vel_time = rospy.get_rostime()
        last_js_time = rospy.Time(0)
        # We set the retry count to 0 initially to make sure that only 
        # if we received at least one sensor package, we are robust 
        # agains a few sensor read failures. For some strange reason, 
        # sensor read failures can occur when switching to full mode 
        # on the Roomba. 
        sensor_read_retry_count = 0 


        while not rospy.is_shutdown():
            last_time = s.header.stamp
            curr_time = rospy.get_rostime()

            # SENSE/COMPUTE STATE
            try:
                if self.sense(s):
                    transform = self.compute_odom(s, last_time, odom)
                # Future-date the joint states so that we don't have
                # to publish as frequently.
                js.header.stamp = curr_time + rospy.Duration(1)
            except select.error:
                # packet read can get interrupted, restart loop to
                # check for exit conditions
                continue

#             except DriverError: 
#                 if sensor_read_retry_count > 0: 
#                     rospy.logwarn('Failed to read sensor package. %d retries left.' % sensor_read_retry_count) 
#                     sensor_read_retry_count -= 1 
#                     continue 
#                 else: 
#                     raise 
            sensor_read_retry_count = self._SENSOR_READ_RETRY_COUNT 

            #TODO -oDE: handle low battery once battery level information is available!!

#             # Reboot Create if we detect that charging is necessary.
#             if s.charging_sources_available > 0 and \
#                    s.oi_mode == 1 and \
#                    s.charging_state in [0, 5] and \
#                    s.charge < 0.93*s.capacity:
#                 rospy.loginfo("going into soft-reboot and exiting driver")
#                 self._robot_reboot()
#                 rospy.loginfo("exiting driver")
#                 break

#             # Reboot Create if we detect that battery is at critical level switch to passive mode.
#             if s.charging_sources_available > 0 and \
#                    s.oi_mode == 3 and \
#                    s.charging_state in [0, 5] and \
#                    s.charge < 0.15*s.capacity:
#                 rospy.loginfo("going into soft-reboot and exiting driver")
#                 self._robot_reboot()
#                 rospy.loginfo("exiting driver")
#                 break

            # PUBLISH STATE
            self.sensor_state_pub.publish(s)
            self.odom_pub.publish(odom)
            if self.publish_tf:
                self.publish_odometry_transform(odom)
            # 1hz, future-dated joint state
            if curr_time > last_js_time + rospy.Duration(1):
                self.joint_states_pub.publish(js)
                last_js_time = curr_time
            self._diagnostics.publish(s)
            if self.imu_used:
                self.sensor_handler.publish_imu(s, last_time)
#             if self._imu:
#                 self._imu.publish(s, last_time)

            # ACT
            if self.req_cmd_vel is not None:
                # check for bumper contact and limit drive command
                req_cmd_vel = self.check_bumpers(s, self.req_cmd_vel)

                # Set to None so we know it's a new command
                self.req_cmd_vel = None
                # reset time for timeout
                last_cmd_vel_time = last_time

            else:
                #zero commands on timeout
                if last_time - last_cmd_vel_time > self.cmd_vel_timeout:
                    last_cmd_vel = 0,0
                # double check bumpers
                req_cmd_vel = self.check_bumpers(s, last_cmd_vel)

            # send command
            if self.move:
                self.drive_cmd(*req_cmd_vel)
            # record command
            last_cmd_vel = req_cmd_vel

            r.sleep()

    def check_bumpers(self, s, cmd_vel):
        # TODO -oDE: handle bumper more acurately. do escape
        
        # Safety: disallow forward motion if bumpers or wheeldrops
        # are activated.
        # TODO: check bumps_wheeldrops flags more thoroughly, and disable
        # all motion (not just forward motion) when wheeldrops are activated
        forward = (cmd_vel[0] + cmd_vel[1]) > 0
        if self.stop_motors_on_bump and s.bumper and forward:
            return (0,0)
        else:
            return cmd_vel

    def compute_odom(self, sensor_state, last_time, odom):
        """
        Compute current odometry.  Updates odom instance and returns tf
        transform. compute_odom() does not set frame ids or covariances in
        Odometry instance.  It will only set stamp, pose, and twist.

        @param sensor_state: Current sensor reading
        @type  sensor_state: Robo50SensorState
        @param last_time: time of last sensor reading
        @type  last_time: rospy.Time
        @param odom: Odometry instance to update.
        @type  odom: nav_msgs.msg.Odometry

        @return: transform
        @rtype: ( (float, float, float), (float, float, float, float) )
        """
        # based on otl_roomba by OTL <t.ogura@gmail.com>

        current_time = sensor_state.header.stamp
        dt = (current_time - last_time).to_sec()

        # On startup, Create can report junk readings
        if abs(sensor_state.distance) > 1.0 or abs(sensor_state.angle) > 1.0:
            raise Exception("Distance, angle displacement too big, invalid readings from robot. Distance: %.2f, Angle: %.2f" % (sensor_state.distance, sensor_state.angle))

        # this is really delta_distance, delta_angle
        d  = sensor_state.distance * self.odom_linear_scale_correction #correction factor from calibration
        angle = sensor_state.angle * self.odom_angular_scale_correction #correction factor from calibration

        x = cos(angle) * d
        y = -sin(angle) * d

        last_angle = self._pos2d.theta
        self._pos2d.x += cos(last_angle)*x - sin(last_angle)*y
        self._pos2d.y += sin(last_angle)*x + cos(last_angle)*y
        self._pos2d.theta += angle

        # Turtlebot quaternion from yaw. simplified version of tf.transformations.quaternion_about_axis
        odom_quat = (0., 0., sin(self._pos2d.theta/2.), cos(self._pos2d.theta/2.))

        # construct the transform
        transform = (self._pos2d.x, self._pos2d.y, 0.), odom_quat

        # update the odometry state
        odom.header.stamp = current_time
        odom.pose.pose   = Pose(Point(self._pos2d.x, self._pos2d.y, 0.), Quaternion(*odom_quat))
        odom.twist.twist = Twist(Vector3(d/dt, 0, 0), Vector3(0, 0, angle/dt))
        if sensor_state.requested_right_velocity == 0 and \
               sensor_state.requested_left_velocity == 0 and \
               sensor_state.distance == 0:
            odom.pose.covariance = ODOM_POSE_COVARIANCE2
            odom.twist.covariance = ODOM_TWIST_COVARIANCE2
        else:
            odom.pose.covariance = ODOM_POSE_COVARIANCE
            odom.twist.covariance = ODOM_TWIST_COVARIANCE

        # return the transform
        return transform

    def publish_odometry_transform(self, odometry):
        self.transform_broadcaster.sendTransform(
            (odometry.pose.pose.position.x, odometry.pose.pose.position.y, odometry.pose.pose.position.z),
            (odometry.pose.pose.orientation.x, odometry.pose.pose.orientation.y, odometry.pose.pose.orientation.z,
             odometry.pose.pose.orientation.w),
             odometry.header.stamp, odometry.child_frame_id, odometry.header.frame_id)

def connected_file():
    return os.path.join(roslib.rosenv.get_ros_home(), 'robo50-connected')

def onShutdown():
    pass
    # TODO -oDE: shutdown robot if necessary

def robo50_main(argv):
    global node
    node = Robo50Node()
    rospy.on_shutdown(onShutdown)
    while not rospy.is_shutdown():
        try:
            # This sleep throttles reconnecting of the driver.  It
            # appears that pyserial does not properly release the file
            # descriptor for the USB port in the event that the Create is
            # unplugged from the laptop.  This file desecriptor prevents
            # the create from reassociating with the same USB port when it
            # is plugged back in.  The solution, for now, is to quickly
            # exit the driver and let roslaunch respawn the driver until
            # reconnection occurs.  However, it order to not do bad things
            # to the Create bootloader, and also to keep relaunching at a
            # minimum, we have a 3-second sleep.
            time.sleep(3.0)
            
            node.start()
            node.spin()

        except Exception as ex:
            msg = "Failed to contact device with error: [%s]."%(ex)
            node._diagnostics.node_status(msg,"error")
            rospy.logerr(msg)

        finally:
            # Driver no longer connected, delete flag from disk
            try:
                os.remove(connected_file())
            except Exception: pass

if __name__ == '__main__':
    robo50_main(sys.argv)
