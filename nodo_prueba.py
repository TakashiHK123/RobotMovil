#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def callback(data):
    print("wooho! ",data)

def listener():

   
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("topic", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()