# Software License Agreement (BSD License)
#
# Copyright (c) 2011, Willow Garage, Inc.
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
#  * Neither the name of the Willow Garage nor the names of its
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

import struct
import logging
import time
import math
import json
import PyKDL

import rospy
import driver

import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs

class Robo50SensorHandler(object):
    
    def __init__(self, robot):
        self._robot = robot
        self._robot.OnDecodeSensorData = self.decodeSensorData
        self._last_encoder_counts = None
        
        self.data_recv = False
        self.compass = {'heading': 0}
        self.accelero = {'x': 0, 'y': 0, 'z': 0}
        self.gyro = {'x': 0, 'y': 0, 'z': 0}
        self.odom = {'left_encoder': 0, 'right_encoder': 0}
        self.motor = {'motor1': 0, 'motor2': 0, 'motor1peak': 0, 'motor2peak': 0}
        
        self.cal_offset = 0.0
        self.orientation = 0.0
        self.cal_buffer = []
        self.cal_buffer_length = 1000
        self.imu_data = sensor_msgs.Imu(header=rospy.Header(frame_id="gyro_link"))
        self.imu_data.orientation_covariance = [1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6]
        self.imu_data.angular_velocity_covariance = [1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6]
        self.imu_data.linear_acceleration_covariance = [-1,0,0,0,0,0,0,0,0]
        # self.gyro_measurement_range = rospy.get_param('~gyro_measurement_range', 150.0) 
        self.gyro_scale_correction = rospy.get_param('~gyro_scale_correction', 1.0)
        self.imu_pub = rospy.Publisher('imu/data', sensor_msgs.Imu)
        self.imu_pub_raw = rospy.Publisher('imu/raw', sensor_msgs.Imu)
      
    def reconfigure(self, config, level): 
        # self.gyro_measurement_range = rospy.get_param('~gyro_measurement_range', 150.0) 
        self.gyro_scale_correction = rospy.get_param('~gyro_scale_correction', 1.0) 
        # rospy.loginfo('self.gyro_measurement_range %f' %self.gyro_measurement_range) 
        self.imu = rospy.get_param('~imu_used', False)
        
        rospy.loginfo("use imu: %s"%(self.imu))
        rospy.loginfo('self.gyro_scale_correction %f' %self.gyro_scale_correction) 

    def decodeSensorData(self, data):
        try:
            if data.get('compass'):
    #                 self.compass.heading = data['compass']['heading']
                self.compass = data['compass']
                
            if data.get('accelero'):
    #                 self.accelero.x = data['accelero']['x']
    #                 self.accelero.y = data['accelero']['y']
    #                 self.accelero.z = data['accelero']['z']
                self.accelero = data['accelero']
                
            if data.get('gyro'):
    #                 self.gyro.x = data['gyro']['x']
    #                 self.gyro.y = data['gyro']['y']
    #                 self.gyro.z = data['gyro']['z']
                self.gyro = data['gyro']
                
            if data.get('odom'):
    #                 self.odom.right_encoder = data['odom']['rightencoder']
    #                 self.odom.left_encoder = data['odom']['leftencoder']
                self.odom = data['odom']
                
            if data.get('motor'):
    #                 self.motor.motor1 = data['motor']['motor1']
    #                 self.motor.motor2 = data['motor']['motor2']
    #                 self.motor.motor1peak = data['motor']['motor1peak']
    #                 self.motor.motor2peak = data['motor']['motor2peak']
                self.motor = data['motor']
    
            self.data_recv = True
        except Exception, e:
            print e
        
        return
          
    def compute_odom(self):
        # The distance and angle calculation sent by the robot seems to
        # be really bad. Re-calculate the values using the raw enconder
        # counts.
        if self._last_encoder_counts:
            count_delta_left = self._normalize_encoder_count(
                self.odom['leftencoder'] - self._last_encoder_counts[0], 0xffff)
            count_delta_right = self._normalize_encoder_count(
                self.odom['rightencoder'] - self._last_encoder_counts[1], 0xffff)
            distance_left = count_delta_left * self._robot.PULSES_TO_M
            distance_right = count_delta_right * self._robot.PULSES_TO_M
            self.distance_left = distance_left
            self.distance_right = distance_right
            self.distance = (distance_left + distance_right) / 2.0
            self.angle = (distance_right - distance_left) / self._robot.WHEEL_SEPARATION
        else:
            self.disance = 0
            self.angle = 0
            self._last_encoder_counts = (self.encoder_counts_left, self.encoder_counts_right)

    def normalize_encoder_count(self, count_delta, maximal_count):
        if count_delta >= maximal_count / 2:
            return count_delta - maximal_count + 1
        elif count_delta <= -maximal_count / 2:
            return count_delta + maximal_count + 1
        return count_delta

    def encode_message(self, message, timestamp):
        message.header = std_msgs.Header(stamp=rospy.Time.from_seconds(timestamp))
        
        try:
            message.distance = self.distance
            message.angle= self.angle
            message.encoder_counts_left = self.encoder_counts_left
            message.encoder_counts_right = self.encoder_counts_right
            message.distance_left = self.distance_left
            message.distance_right = self.distance_right
            
            message.requested_right_velocity = self._robot.current_right_velocity
            message.requested_left_velocity = self._robot.current_left_velocity
            
            message.imu.heading = self.compass['heading']
            message.imu.gyro.x = self.gyro['x']
            message.imu.gyro.y = self.gyro['y']
            message.imu.gyro.z = self.gyro['z']
            message.imu.accelero.x = self.accelero['x']
            message.imu.accelero.y = self.accelero['y']
            message.imu.accelero.z = self.accelero['z']
            
            message.bumper = False
    #         message.bumper = self.bumper

            message.motor1 = self.motor['motor1']
            message.motor2 = self.motor['motor2']
            message.motor1peak = self.motor['motor1peak']
            message.motor2peak = self.motor['motor2peak']
            
            #TODO -oDE: requested velocities von arduino?
        except:
            pass
        
        return message
      
    def getSensorData(self, sensor_state):
        # sensor data is getting streamed from the robot, just use the last received data
        timestamp = time.time()
        if self.data_recv:
            self.compute_odom()
            self.data_recv = False
        self.encode_message(sensor_state, timestamp)
        
        return True
        # else:
        #     return False

    def update_imu_calibration(self, sensor_state):
        # check if we're not moving and update the calibration offset
        # to account for any calibration drift due to temperature
        if sensor_state.requested_right_velocity == 0 and \
               sensor_state.requested_left_velocity == 0 and \
               sensor_state.distance == 0:
        
            self.cal_buffer.append(self.sensor_state.gyro.z)
            if len(self.cal_buffer) > self.cal_buffer_length:
                del self.cal_buffer[:-self.cal_buffer_length]
            self.cal_offset = sum(self.cal_buffer)/len(self.cal_buffer)

    def publish_imu(self, sensor_state, last_time):
        if self.cal_offset == 0:
            self.orient_offset = sensor_state.heading
            return

        current_time = sensor_state.header.stamp
        dt = (current_time - last_time).to_sec()
        past_orientation = self.orientation
        # self.orientation = -1.0 * (self.data['compass']['heading'] - self.orient_offset)*(math.pi/180.0)
        self.imu_data.header.stamp =  sensor_state.header.stamp
        self.imu_data.angular_velocity.z = float(sensor_state.gyro.z - self.cal_offset)*(math.pi/180.0)*self.gyro_scale_correction

        self.orientation += self.imu_data.angular_velocity.z * dt
        #print orientation
        (self.imu_data.orientation.x, self.imu_data.orientation.y, self.imu_data.orientation.z, self.imu_data.orientation.w) = PyKDL.Rotation.RotZ(self.orientation).GetQuaternion()
        self.imu_pub.publish(self.imu_data)

        self.imu_data.header.stamp =  sensor_state.header.stamp
        self.imu_data.angular_velocity.z  = float(sensor_state.gyro.z)*(math.pi/180.0)*self.gyro_scale_correction
        #sign change
        self.imu_data.angular_velocity.z = -1.0*self.imu_data.angular_velocity.z
        raw_orientation = past_orientation + self.imu_data.angular_velocity.z * dt
        #print orientation
        (self.imu_data.orientation.x, self.imu_data.orientation.y, self.imu_data.orientation.z, self.imu_data.orientation.w) = PyKDL.Rotation.RotZ(raw_orientation).GetQuaternion()
        self.imu_pub_raw.publish(self.imu_data)
