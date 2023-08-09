#!/usr/bin/env python3
import rospy
from std_msgs.msg import String


def msgCallback(data):
    rospy.loginfo("I heard %s",data.data)

if __name__=='__main__':
    rospy.init_node("listener",anonymous=True)
    rospy.Subscriber("chatter", String, msgCallback)
    rospy.spin()