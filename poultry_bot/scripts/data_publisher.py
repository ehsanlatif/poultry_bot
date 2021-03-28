#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

# import rospy
# # from std_msgs.msg import String
# from poultry_bot.msg import Temperature
# import random as rand

# def talker():
#     pub = rospy.Publisher('temperature', Temperature, queue_size=10)
#     rospy.init_node('talker', anonymous=True)
#     rate = rospy.Rate(10) # 10hz
#     while not rospy.is_shutdown():
#         #hello_str = "hello world %s" % rospy.get_time()
#         temperature = rand.randint(0,100)
#         rospy.loginfo(temperature)
#         pub.publish(temperature)
#         rate.sleep()


#!/usr/bin/env python
#import roslib; roslib.load_manifest('numpy_tutorials') #not sure why I need this
import rospy
from std_msgs.msg import String
from poultry_bot.msg import Proximity,Gesture,Color,Light,Temperature,Pressure,Altitude,Magnetic,Acceleration,Gyro,Humidity,Sound
import serial

ser = serial.Serial('/dev/ttyACM0', 115200)

def publisher():
 while not rospy.is_shutdown():
  for i in range(6):
    input_data = ser.readline().strip()
    data = input_data.split(":")[0]
    value = input_data.split(":")[1]
    if data == "Proximity":
      print value
      proximity= int(value)
      rospy.loginfo(proximity)
      pub_proximity.publish(proximity)
    elif data == "Gesture":
      print value
      gesture= int(value)
      rospy.loginfo(gesture)
      pub_gesture.publish(gesture)
    elif data == "Color":
      print value
      color = Color()
      color.red = int(value.split(" ")[1])
      color.green = int(value.split(" ")[2])
      color.blue = int(value.split(" ")[3])
      rospy.loginfo(color)
      pub_color.publish(color)
      #light
      light= int(value.split(" ")[3])
      rospy.loginfo(light)
      pub_light.publish(light)
    elif data == "Temperature":
      print value
      temperature= float(value)
      rospy.loginfo(temperature)
      pub_temperature.publish(temperature)
    elif data == "Barometric pressure":
      print value
      pressure= float(value)
      rospy.loginfo(pressure)
      pub_pressure.publish(pressure)
    elif data == "Altitude":
      print value
      altitude= float(value)
      rospy.loginfo(altitude)
      pub_altitude.publish(altitude)
    elif data == "Magnetic":
      print value
      magnetic = Magnetic()
      magnetic.x_magnetic = float(value.split(" ")[1])
      magnetic.y_magnetic = float(value.split(" ")[2])
      magnetic.z_magnetic = float(value.split(" ")[3])
      rospy.loginfo(magnetic)
      pub_magnetic.publish(magnetic)
    elif data == "Acceleration":
      print value
      acceleration = Acceleration()
      acceleration.x_acceleration = float(value.split(" ")[1])
      acceleration.y_acceleration = float(value.split(" ")[2])
      acceleration.z_acceleration = float(value.split(" ")[3])
      rospy.loginfo(acceleration)
      pub_acceleration.publish(acceleration)
    elif data == "Gyro":
      print value
      gyro = Gyro()
      gyro.x_gyro= float(value.split(" ")[1])
      gyro.y_gyro = float(value.split(" ")[2])
      gyro.z_gyro = float(value.split(" ")[3])
      rospy.loginfo(gyro)
      pub_gyro.publish(gyro)
    elif data == "Humidity":
      print value
      humidity= float(value)
      rospy.loginfo(humidity)
      pub_humidity.publish(humidity)
    elif data == "Sound level":
      print value
      sound= int(value)
      rospy.loginfo(sound)
      pub_sound.publish(sound)

  rospy.sleep(0.5)


if __name__ == '__main__':
  try:
    pub_proximity = rospy.Publisher('proximity', Light,queue_size=10)
    pub_gesture = rospy.Publisher('gesture', Gesture,queue_size=10)
    pub_color = rospy.Publisher('color', Color,queue_size=10)
    pub_light = rospy.Publisher('light', Light,queue_size=10)
    pub_temperature = rospy.Publisher('temperature', Temperature,queue_size=10)
    pub_pressure = rospy.Publisher('pressure', Pressure,queue_size=10)
    pub_altitude = rospy.Publisher('altitude', Altitude,queue_size=10)
    pub_magnetic = rospy.Publisher('magnetic', Magnetic,queue_size=10)
    pub_acceleration = rospy.Publisher('acceleration', Acceleration,queue_size=10)
    pub_gyro = rospy.Publisher('gyro', Gyro,queue_size=10)
    pub_humidity = rospy.Publisher('humidity', Humidity,queue_size=10)
    pub_sound = rospy.Publisher('sound', Sound,queue_size=10)
    rospy.init_node('publisher')
    publisher()
  except rospy.ROSInterruptException:
    pass