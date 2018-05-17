#!/usr/bin/env python
# http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Simple%20Action%20Server%20using%20the%20Execute%20Callback%20%28Python%29
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from sensor_msgs.msg import JointState


class RobotTrajectoryFollower(object):
    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                FollowJointTrajectoryAction,
                                                execute_cb=self.execute_cb)

        # Publisher of commands
        self.pub = rospy.Publisher('command/joint_states', JointState,
                                   queue_size=10)

        self._as.start()

    def execute_cb(self, data):
        r = rospy.Rate(20)
        numOfPoints = len(data.trajectory.points)
        jointState = JointState()
        print("Received Joint Trajectory with " + str(numOfPoints) +
              " points")
        if numOfPoints > 0:
            jointState.name = data.trajectory.joint_names
            for i in range(numOfPoints):
                jointState.position = data.trajectory.\
                    points[i].positions
                self.pub.publish(jointState)
                r.sleep()
        self._as.set_succeeded()


if __name__ == '__main__':
    rospy.init_node('action_server')
    server = RobotTrajectoryFollower('joint_trajectory_action')
    rospy.spin()
