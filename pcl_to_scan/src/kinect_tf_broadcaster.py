#!/usr/bin/env python  
import roslib
roslib.load_manifest('pcl_to_scan')
import rospy
import std_msgs

# Give ourselves the ability to run a dynamic reconfigure server.
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
# Import dynamic reconfigure variables.
from pcl_to_scan.cfg import kinect_tf_broadcaster_paramsConfig as Params

import tf

class kinect_tf_broadcaster():
    def __init__(self):
        self.roll = rospy.get_param("~roll", 0.0)
        self.pitch = rospy.get_param("~pitch", 0.0)
        
        self.server = DynamicReconfigureServer(Params, self.reconfigure)
        
        #print "R: %f P: %f" % (roll, pitch)
        #rospy.Subscriber('/cur_tilt_angle', std_msgs.msg.Float64, handle_kinect_tilt)
        #rospy.spin()
        self.angle_0 = tf.transformations.quaternion_from_euler(0, 0, 0)
        self.angle_1 = tf.transformations.quaternion_from_euler(-1.57, 0, -1.57)
        self.angle_2 = tf.transformations.quaternion_from_euler(-self.roll, -self.pitch, 0)
        while not rospy.is_shutdown():
            stamp = rospy.Time.now() + rospy.Duration(0.3)
            tf.TransformBroadcaster().sendTransform( (0, 0, 0.036), self.angle_0, stamp, "/openni_camera", "/kinect_rotated_base")
            tf.TransformBroadcaster().sendTransform( (0, -0.02, 0), self.angle_0, stamp, "/openni_depth_frame", "/openni_camera")
            tf.TransformBroadcaster().sendTransform( (0, -0.04, 0), self.angle_0, stamp, "/openni_rgb_frame", "/openni_camera")
            tf.TransformBroadcaster().sendTransform( (0, 0, 0),     self.angle_1, stamp, "/openni_depth_optical_frame", "/openni_depth_frame")
            tf.TransformBroadcaster().sendTransform( (0, 0, 0),     self.angle_1, stamp, "/openni_rgb_optical_frame", "/openni_rgb_frame")
            tf.TransformBroadcaster().sendTransform( (0, 0, 0),     self.angle_2, stamp, "/kinect_rotated_base", "/kinect_base")
            tf.TransformBroadcaster().sendTransform( (-0.077, 0, 0.35),  self.angle_0, stamp, "/kinect_base", "/base_link")
            rospy.sleep(0.2)

    # Create a callback function for the dynamic reconfigure server.
    def reconfigure(self, config, level):
        # Fill in local variables with values received from dynamic reconfigure clients (typically the GUI).
        self.roll = config["roll"]
        self.pitch = config["pitch"]
        self.angle_2 = tf.transformations.quaternion_from_euler(-self.roll, -self.pitch, 0)
        # Return the new variables.
        return config

if __name__ == '__main__':
    rospy.init_node('kinect_tf_broadcaster')
    
    try:
        ktb = kinect_tf_broadcaster()
    except rospy.ROSInterruptException: pass
    
