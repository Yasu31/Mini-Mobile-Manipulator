#!/usr/bin/env python
# http://wiki.ros.org/tf/Tutorials/Adding%20a%20frame%20%28Python%29
import rospy
import tf
# requires YAML, rospkg, catkin_pkg

if __name__=='__main__':
    rospy.init_node('broadcaster_marker_to_hand', anonymous=True)
    br=tf.TransformBroadcaster()
    rate=rospy.Rate(10)
    while not rospy.is_shutdown():
        br.sendTransform((0.0, 0.0, 0.05),(0,0,0,1),rospy.Time.now(),"kxr_hand","ar_marker_0")
        rate.sleep()
