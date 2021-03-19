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
from poultry_bot.msg import Temperature,Light,Acceleration,Angle,Humidity,Sound
import serial

ser = serial.Serial('/dev/ttyACM0', 115200)

def publisher():
 while not rospy.is_shutdown():
  for i in range(6):
    input_data = ser.readline().strip()
    data = input_data.split(":")[0]
    value = input_data.split(":")[1]
    if data == "light":
      print value
      light= int(value)
      rospy.loginfo(light)
      pub_light.publish(light)
    elif data == "temperature":
      print value
      temperature= float(value)
      rospy.loginfo(temperature)
      pub_temperature.publish(temperature)
    elif data == "acceleration":
      print value
      acceleration = Acceleration()
      acceleration.x_acceleration = float(value.split(" ")[1])
      acceleration.y_acceleration = float(value.split(" ")[2])
      acceleration.z_acceleration = float(value.split(" ")[3])
      rospy.loginfo(acceleration)
      pub_acceleration.publish(acceleration)
    elif data == "angle":
      print value
      angle = Angle()
      angle.xy_angle = float(value.split(" ")[1])
      angle.yz_angle = float(value.split(" ")[2])
      angle.zx_angle = float(value.split(" ")[3])
      rospy.loginfo(angle)
      pub_angle.publish(angle)
    elif data == "humidity":
      print value
      humidity= int(value)
      rospy.loginfo(humidity)
      pub_humidity.publish(humidity)
    elif data == "Sound level":
      print value
      sound= float(value)
      rospy.loginfo(sound)
      pub_sound.publish(sound)

  rospy.sleep(0.5)


if __name__ == '__main__':
  try:
    pub_light = rospy.Publisher('light', Light,queue_size=10)
    pub_temperature = rospy.Publisher('temperature', Temperature,queue_size=10)
    pub_acceleration = rospy.Publisher('acceleration', Acceleration,queue_size=10)
    pub_angle = rospy.Publisher('angle', Angle,queue_size=10)
    pub_humidity = rospy.Publisher('humidity', Humidity,queue_size=10)
    pub_sound = rospy.Publisher('sound', Sound,queue_size=10)
    rospy.init_node('publisher')
    publisher()
  except rospy.ROSInterruptException:
    pass