#!/usr/bin/env python

import rospy
from firefly_mapping.msg import ImageWithPose
from sensor_msgs.msg import Image

if __name__ == '__main__':
    rospy.init_node('projection_tester', anonymous=True)
    pub = rospy.Publisher('image_to_project', ImageWithPose, queue_size=10)
    pub1 = rospy.Publisher('image', Image, queue_size=10)

    # msg = ImageWithPose()
    # msg.pose.position.x = 0.151
    # msg.pose.position.y = -0.011
    # msg.pose.position.z = 9.865
    # msg.pose.orientation.x = -0.587
    # msg.pose.orientation.y = 0.617
    # msg.pose.orientation.z = -0.381
    # msg.pose.orientation.w = 0.359
    # msg.image.width = 640
    # msg.image.height = 480
    # msg.image.data = [0] * (640*480)
    msg = ImageWithPose()
    msg.pose.position.x = 0.151
    msg.pose.position.y = -0.011
    msg.pose.position.z = 29.865
    msg.pose.orientation.x = -0.689
    msg.pose.orientation.y = 0.725
    msg.pose.orientation.z = -0.001
    msg.pose.orientation.w = -0.002
    msg.image.width = 200
    msg.image.height = 150
    msg.image.data = [0] * (200*150)
    # msg = ImageWithPose()
    # msg.pose.position.x = 0.151
    # msg.pose.position.y = -0.011
    # msg.pose.position.z = 29.865
    # msg.pose.orientation.x = -0.683
    # msg.pose.orientation.y = 0.719
    # msg.pose.orientation.z = -0.096
    # msg.pose.orientation.w = 0.088
    # msg.image.width = 640
    # msg.image.height = 480
    # msg.image.data = [0] * (640*480)

    for i in range(50, 100):
        for j in range(75, 125):
            msg.image.data[i*200 + j] = 255
    msg.image.encoding = 'mono8'
    rospy.sleep(1.0)
    pub.publish(msg)
    pub1.publish(msg.image)
