#! /usr/bin/python
#***********************************************************
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
#

# Based on turtlebot_calibration by Wim Meeussen

from __future__ import with_statement

import roslib; roslib.load_manifest('elektron_calibration')
import yaml
import PyKDL
import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from elektron_calibration.msg import ScanDistAngle
from math import *
import threading


def quat_to_angle(quat):
    rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
    return rot.GetRPY()[2]
        
def normalize_angle(angle):
    res = angle
    while res > pi:
        res -= 2.0*pi
    while res < -pi:
        res += 2.0*pi
    return res


class CalibrateRobot:
    def __init__(self):
        self.lock = threading.Lock()
        self.sub_imu  = rospy.Subscriber('imu/data', Imu, self.imu_cb)
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.odom_cb)
        self.sub_scan = rospy.Subscriber('scan_dist_angle', ScanDistAngle, self.scan_cb)
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist)
        self.imu_time = rospy.Time()
        self.odom_time = rospy.Time()
        self.scan_time = rospy.Time()
        
        # params
        self.inital_wall_angle = rospy.get_param("inital_wall_angle", 0.1)
        self.inital_wall_dist = rospy.get_param("inital_wall_angle", 3.0)
        self.imu_calibrate_time = rospy.get_param("imu_calibrate_time", 10.0)
        


    def calibrate_rot(self, speed, imu_drift=0):
        # rotate 360 degrees
        (imu_start_angle, odom_start_angle, scan_start_angle, 
         imu_start_time, odom_start_time, scan_start_time,
         xx, yy, dd) = self.sync_timestamps()
        last_angle = odom_start_angle
        turn_angle = 0
        while turn_angle < 15.0/8*pi:
            if rospy.is_shutdown():
                return
            cmd = Twist()
            cmd.angular.z = speed
            self.cmd_pub.publish(cmd)
            rospy.sleep(0.025)
            with self.lock:
                delta_angle = normalize_angle(self.odom_angle - last_angle)
            turn_angle += delta_angle
            last_angle = self.odom_angle
        self.cmd_pub.publish(Twist())

        rospy.sleep(1.0)

        (imu_end_angle, odom_end_angle, scan_end_angle,
         imu_end_time, odom_end_time, scan_end_time,
         xx, yy, dd) = self.sync_timestamps()
        imu_delta = 2*pi + normalize_angle(imu_end_angle - imu_start_angle) - imu_drift*(imu_end_time - imu_start_time).to_sec()
        odom_delta = 2*pi + normalize_angle(odom_end_angle - odom_start_angle)
        scan_delta = 2*pi + normalize_angle(scan_end_angle - scan_start_angle)
        rospy.loginfo('Imu error: %f percent'%(100.0*((imu_delta/scan_delta)-1.0)))
        rospy.loginfo('Odom error: %f percent'%(100.0*((odom_delta/scan_delta)-1.0)))
        return (imu_delta/scan_delta, odom_delta/scan_delta)
    
    def calibrate_lin(self, speed):
        # rotate 360 degrees
        (imu_end_angle, odom_end_angle, scan_end_angle,
         imu_end_time, odom_end_time, scan_end_time,
         xx, yy, dd) = self.sync_timestamps()
        last_dist = dd
        dist = 0.0
        last_x = xx
        last_y = yy
        to_travel = dd - 0.5
        while dist < to_travel:
            if rospy.is_shutdown():
                return
            cmd = Twist()
            cmd.linear.x = speed
            self.cmd_pub.publish(cmd)
            rospy.sleep(0.025)
            with self.lock:
                dx = self.odom_x - last_x
                dy = self.odom_y - last_y
                last_x = self.odom_x
                last_y = self.odom_y
                delta_dist = sqrt(dx*dx+dy*dy)
            dist += delta_dist
        self.cmd_pub.publish(Twist())

        rospy.sleep(1.0)

        (imu_end_angle, odom_end_angle, scan_end_angle,
         imu_end_time, odom_end_time, scan_end_time,
         e_xx, e_yy, e_dd) = self.sync_timestamps()
        scan_delta = dd - e_dd
        sx = e_xx - xx
        sy = e_yy - yy
        odom_delta = sqrt(sx*sx+sy*sy)
        rospy.loginfo('Odom linear error: %f percent'%(100.0*((odom_delta/scan_delta)-1.0)))
        return odom_delta/scan_delta





    def imu_drift(self):
        # estimate imu drift
        rospy.loginfo('Estimating imu drift')
        (imu_start_angle, odom_start_angle, scan_start_angle, 
         imu_start_time, odom_start_time, scan_start_time,
         xx, yy, dd) = self.sync_timestamps()
        rospy.sleep(self.imu_calibrate_time)
        (imu_end_angle, odom_end_angle, scan_end_angle,
         imu_end_time, odom_end_time, scan_end_time,
         xx, yy, dd) = self.sync_timestamps()
        imu_drift = normalize_angle(imu_end_angle - imu_start_angle) / ((imu_end_time - imu_start_time).to_sec())
        rospy.loginfo(' ... imu drift is %f degrees per second'%(imu_drift*180.0/pi))
        return imu_drift


    def align(self, scale = 1.0):
        rospy.loginfo("Aligning base with wall")
        with self.lock:
            angle = self.scan_angle
        cmd = Twist()

        while angle < -self.inital_wall_angle*scale or angle > self.inital_wall_angle*scale:
            if rospy.is_shutdown():
                exit(0)
            if angle > 0:
                cmd.angular.z = -0.1*scale
            else:
                cmd.angular.z = 0.1*scale
            self.cmd_pub.publish(cmd)
            rospy.sleep(0.025)
            with self.lock:
                angle = self.scan_angle
                
    def go_back(self):
        rospy.loginfo("Aligning base with wall")
        with self.lock:
            dist = self.scan_dist
        cmd = Twist()

        while dist < self.inital_wall_dist:
            if rospy.is_shutdown():
                exit(0)
            
            if (self.inital_wall_dist - dist > 0.5):
                cmd.linear.x = -0.15
            else:
                cmd.linear.x = -0.05
                
            self.cmd_pub.publish(cmd)
            rospy.sleep(0.025)
            with self.lock:
                dist = self.scan_dist





    def sync_timestamps(self, start_time=None):
        if not start_time:
            start_time = rospy.Time.now() + rospy.Duration(0.5)
        while not rospy.is_shutdown():
            rospy.sleep(0.3)
            with self.lock:
                if self.imu_time < start_time:
                    rospy.loginfo("Still waiting for imu")
                elif self.odom_time < start_time:
                    rospy.loginfo("Still waiting for odom")
                elif self.scan_time < start_time:
                    rospy.loginfo("Still waiting for scan")
                else:
                    return (self.imu_angle, self.odom_angle, self.scan_angle,
                            self.imu_time, self.odom_time, self.scan_time,
                            self.odom_x, self.odom_y, self.scan_dist)
        exit(0)
        

    def imu_cb(self, msg):
        with self.lock:
            angle = quat_to_angle(msg.orientation)
            self.imu_angle = angle
            self.imu_time = msg.header.stamp

    def odom_cb(self, msg):
        with self.lock:
            angle = quat_to_angle(msg.pose.pose.orientation)
            self.odom_angle = angle
            self.odom_x = msg.pose.pose.position.x
            self.odom_y = msg.pose.pose.position.y
            self.odom_time = msg.header.stamp

    def scan_cb(self, msg):
        with self.lock:
            self.scan_angle = msg.angle
            self.scan_dist = msg.dist
            self.scan_time = msg.header.stamp



def main():
    rospy.init_node('scan_to_angle')
    robot = CalibrateRobot()
    
    imu_drift = robot.imu_drift()
    imu_corr = []
    odom_corr = []
    odom_lin_corr = []
    for speed in (0.3, 0.7, 1.0):
        robot.align()
        rospy.sleep(1.0)
        (imu, odom) = robot.calibrate_rot(speed, imu_drift)
        imu_corr.append(imu)
        odom_corr.append(odom)
        
    for speed in (0.05, 0.1, 0.2):
        robot.align(0.3)
        rospy.sleep(1.0)
        robot.go_back()
        rospy.sleep(2.0)
        odom_lin = robot.calibrate_lin(speed)
        odom_lin_corr.append(odom_lin)
        
    imu_res = 1.0/(sum(imu_corr)/len(imu_corr))
    odom_res = 1.0/(sum(odom_corr)/len(odom_corr))
    odom_lin_res = 1.0/(sum(odom_lin_corr)/len(odom_lin_corr))
    rospy.loginfo("Multiply the 'gyroscope/rot_scale' parameter with %f"%imu_res)
    rospy.loginfo("Multiply the 'elektron_base_node/rot_scale' parameter with %f"%odom_res)
    rospy.loginfo("Multiply the 'elektron_base_node/lin_scale' parameter with %f"%odom_lin_res)


if __name__ == '__main__':
    main()
