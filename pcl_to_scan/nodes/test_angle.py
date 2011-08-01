#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2011, Robot Control and Pattern Recognition Group, Warsaw University of Technology
#
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the <organization> nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


import roslib
roslib.load_manifest('pcl_to_scan')

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

from math import *

class TestAngle:
    def __init__(self):
        self.sub = rospy.Subscriber('imu', Imu, self.imu_cb)
        
        self.pub = rospy.Publisher('tilt_angle', Float64, None, False, True)
        
        self.sum_p = 0.0
        self.sum_r = 0.0
        self.cnt = 0
        self.measure = False
        self.duration = 10
        

    def setTilt(self):
        print "Set tilt..."
        self.pub.publish(0.0)
        rospy.sleep(5.0)
        if not rospy.is_shutdown():
            self.pub.publish(-20.0)

    def doMeasure(self):
        print "Do measure..."
        self.measure = True
        self.start_time = rospy.Time.now()
        rospy.sleep(self.duration)
        self.measure = False
        if not rospy.is_shutdown():
            rospy.loginfo("Current Kinect angle: pitch = %f, roll = %f" % (self.sum_p/self.cnt, self.sum_r/self.cnt))
            rospy.loginfo("[in degs: pitch = %f, roll = %f]" % (self.sum_p/self.cnt*180/pi, self.sum_r/self.cnt*180/pi))

    def imu_cb(self, msg):
        if (self.measure):
            x = msg.linear_acceleration.x
            y = msg.linear_acceleration.y
            z = msg.linear_acceleration.z
            pitch = asin(z/9.81)
            roll = asin(x/9.81)
            self.sum_p += pitch
            self.sum_r += roll
            self.cnt += 1
            #print "P: %10.7f    R: %10.7f  [P: %10.7f    R: %10.7f]" % (pitch, roll, self.sum_p/self.cnt, self.sum_r/self.cnt)


def main():
    rospy.init_node('test_angle')
    tester = TestAngle()
    tester.setTilt()
    print "Wait..."
    rospy.sleep(5.0)
    tester.doMeasure()
    rospy.spin()


if __name__ == '__main__':
    main()
