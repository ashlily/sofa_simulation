#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import *

def talker():
    pub = rospy.Publisher('sofa_chatter', Float64MultiArray, queue_size=100)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(50) # 10hz
    while not rospy.is_shutdown():
        position_cmds =[0.001 for i in range(3)]#need to change according to the manipulation_point number
        data_msg = Float64MultiArray()
        data_msg.data = position_cmds
        pub.publish(data_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

