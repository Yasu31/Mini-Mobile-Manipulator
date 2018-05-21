#! /usr/bin/env python
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import rospkg


def img_callback(msg):
    cv_image=bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    filename = str(msg.header.stamp.secs)
    path = rospack.get_path("mayfes") + "/bot/img/" + filename + ".jpg"
    print("saving image to ", path)
    cv2.imwrite(path, cv_image)
    
rospack = rospkg.RosPack()
bridge = CvBridge()
rospy.init_node("image_saver", anonymous=True)
rospy.Subscriber("/save_image", Image, img_callback)
print("Image Saver node is up and running.")
rospy.spin()
