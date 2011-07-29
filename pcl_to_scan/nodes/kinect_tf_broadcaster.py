#!/usr/bin/env python  
import roslib
roslib.load_manifest('pcl_to_scan')
import rospy
import std_msgs

import tf

angle = -20.0

if __name__ == '__main__':
    rospy.init_node('kinect_tf_broadcaster')
    #rospy.Subscriber('/cur_tilt_angle', std_msgs.msg.Float64, handle_kinect_tilt)
    #rospy.spin()
    while not rospy.is_shutdown():
        tf.TransformBroadcaster().sendTransform( (0, 0, 0.036), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now() + rospy.Time.Duration(0.3), "/openni_camera", "/kinect_rotated_base")
        tf.TransformBroadcaster().sendTransform( (0, -0.02, 0), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now() + rospy.Time.Duration(0.3), "/openni_depth_frame", "/openni_camera")
        tf.TransformBroadcaster().sendTransform( (0, -0.04, 0), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now() + rospy.Time.Duration(0.3), "/openni_rgb_frame", "/openni_camera")
        tf.TransformBroadcaster().sendTransform( (0, 0, 0), tf.transformations.quaternion_from_euler(-1.57, 0, -1.57), rospy.Time.now() + rospy.Time.Duration(0.3), "/openni_depth_optical_frame", "/openni_depth_frame")
        tf.TransformBroadcaster().sendTransform( (0, 0, 0), tf.transformations.quaternion_from_euler(-1.57, 0, -1.57), rospy.Time.now() + rospy.Time.Duration(0.3), "/openni_rgb_optical_frame", "/openni_rgb_frame")
        tf.TransformBroadcaster().sendTransform( (0, 0, 0), tf.transformations.quaternion_from_euler(0, -angle * 3.14 / 180, 0), rospy.Time.now() + rospy.Time.Duration(0.3), "/kinect_rotated_base", "/kinect_base")
        rospy.sleep(0.05)
