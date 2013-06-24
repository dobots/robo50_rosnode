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

#Melonee Wise mwise@willowgarage.com

import rospy
import math
import copy
import sensor_msgs.msg
import PyKDL
import json
import time
from serial_gateway import SerialGateway

PULSES_TO_M = 0
#TODO -oDE: define PULSES TO METER

MAX_WHEEL_SPEED = 0
WHEEL_SEPARATION = 0
#TODO -oDE: define MAX_WHEEL_SPEED and WHEEL_SEPARATION

HEADER = 0xA5

SENSOR_DATA     = 0
DRIVE_COMMAND   = 1
MOTOR_COMMAND   = 2
CONTROL_COMMAND = 3
DISCONNECT      = 4

class DriverError(Exception):
  pass

def _OnDecodeSensorData(data):
    print(data)

class Robo50():
    
    mTID = 0

    def __init__(self, sensorDataHandler = _OnDecodeSensorData):
        self.OnDecodeSensorData = sensorDataHandler
        self.current_right_velocity = 0
        self.current_left_velocity = 0
                
    def start(self, port='/dev/ttyACM0', baudrate=115200):
        self.serialHandler = SerialGateway(port, baudrate)
        self.serialHandler.start()

    def stop(self):
        self.serialHandler.stop()
        
    def control(self):
        pass
        #TODO -oDE: add commands if needed for setting the robo50 to listen for control commands
    
    def newCommand(self, type):
        command = {'header': {}, 'data': {}}
        command['header']['id'] = HEADER
        command['header']['timestamp'] = self.mTID
        command['header']['type'] = type
        self.mTID += 1
        return command

    def direct_drive(self, velocity_left, velocity_right):
        drive_cmd = self.newCommand(DRIVE_COMMAND)
        drive_cmd['data']['left'] = velocity_left
        drive_cmd['data']['right'] = velocity_right
        self.send(json.dumps(drive_cmd))
        
        self.current_right_velocity = velocity_right
        self.current_left_velocity = velocity_left
        
    def setMotor(self, motor_id, direction, speed):
        motor_cmd = self.newCommand(MOTOR_COMMAND)
        motor_cmd['data']['motor_id'] = motor_id
        motor_cmd['data']['direction'] = direction
        motor_cmd['data']['speed'] = speed
        self.send(json.dumps(motor_cmd))
        
    def enableControl(self, enabled):
        control_cmd = self.newCommand(CONTROL_COMMAND)
        control_cmd['data']['enabled'] = enabled
        self.send(json.dumps(control_cmd))
        
    def disconnect(self):
        disconnect_cmd = self.newCommand(DISCONNECT)
        self.send(json.dumps(disconnect_cmd))

    def send(self, message):
        self.serialHandler.write(message)

    def decode(self, message):
        print "decoding...."
        try:
            data = json.loads(message)
            
            if data['header']['type'] == SENSOR_DATA:
                self.OnDecodeSensorData(data)
        except:
            pass
          
    def handleSerial(self, line):
        print "handle serial..."
        if (len(line) > 0):
            self.decode(line)
