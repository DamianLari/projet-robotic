#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def talker():
    pub = rospy.Publisher('le_twist', Twist, queue_size=10)
    rospy.init_node('le_twist', anonymous=True)
    rate = rospy.Rate(10) # 10Hz

    while not rospy.is_shutdown():
        twist = Twist()
        twist.linear.x = 0.07478768215979242
        twist.linear.y = -1.6707890205728824
        twist.linear.z = -0.08379452168509689
        twist.angular.x = 0.11094173978058482
        twist.angular.y = 1.0138232898682054
        twist.angular.z = 1.5199572022419752

        rospy.loginfo(twist)
        pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
