#!/usr/bin/env python  
import roslib
roslib.load_manifest('pcl_to_scan')
import rospy
import std_msgs

import tf

if __name__ == '__main__':
    rospy.init_node('kinect_tf_broadcaster')
    
    roll = rospy.get_param("~roll", 0.0)
    pitch = rospy.get_param("~pitch", 0.0) 
    
    #rospy.Subscriber('/cur_tilt_angle', std_msgs.msg.Float64, handle_kinect_tilt)
    #rospy.spin()
    angle_0 = tf.transformations.quaternion_from_euler(0, 0, 0)
    angle_1 = tf.transformations.quaternion_from_euler(-1.57, 0, -1.57)
    angle_2 = tf.transformations.quaternion_from_euler(roll, pitch, 0)
    while not rospy.is_shutdown():
        stamp = rospy.Time.now() + rospy.Duration(0.3)
        tf.TransformBroadcaster().sendTransform( (0, 0, 0.036), angle_0, stamp, "/openni_camera", "/kinect_rotated_base")
        tf.TransformBroadcaster().sendTransform( (0, -0.02, 0), angle_0, stamp, "/openni_depth_frame", "/openni_camera")
        tf.TransformBroadcaster().sendTransform( (0, -0.04, 0), angle_0, stamp, "/openni_rgb_frame", "/openni_camera")
        tf.TransformBroadcaster().sendTransform( (0, 0, 0),     angle_1, stamp, "/openni_depth_optical_frame", "/openni_depth_frame")
        tf.TransformBroadcaster().sendTransform( (0, 0, 0),     angle_1, stamp, "/openni_rgb_optical_frame", "/openni_rgb_frame")
        tf.TransformBroadcaster().sendTransform( (0, 0, 0),     angle_2, stamp, "/kinect_rotated_base", "/kinect_base")
        tf.TransformBroadcaster().sendTransform( (-0.077, 0, 0.35),  angle_0, stamp, "/kinect_base", "/base_link")
        rospy.sleep(0.2)
