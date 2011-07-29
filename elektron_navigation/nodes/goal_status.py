#! /usr/bin/env python

import roslib; roslib.load_manifest('elektron_navigation')
import rospy
from actionlib_msgs.msg import GoalStatusArray

from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient


state = -1
state_info = ""

def goal_status():
    global state
    global state_info
    sub  = rospy.Subscriber('move_base/status', GoalStatusArray, status_cb)
    soundhandle = SoundClient()
    last_state = -1

    snd_failed = rospy.get_param("~snd_failed", "")
    snd_accepted = rospy.get_param("~snd_accepted", "")
    snd_success = rospy.get_param("~snd_success", "")

    while not rospy.is_shutdown():
        
       # print "s: %d, l: %d" % (state, last_state)
        
        if state != last_state and last_state != -1:
            # Failed
            if state == 4:
                soundhandle.playWave(snd_failed)
            # Accepted
            if state == 1:
                soundhandle.playWave(snd_accepted)
            # Success
            if state == 3:
                soundhandle.playWave(snd_success)
            
            rospy.loginfo("State changed to: [%d] %s" % (state, state_info))
                
        last_state = state
        rospy.sleep(2)
        
        
def status_cb(msg):
    global state
    global state_info
    if (len(msg.status_list) > 0):
        state = msg.status_list[0].status
        state_info = msg.status_list[0].text

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('goal_status')
        goal_status()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"