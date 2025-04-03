#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def monitor_cancellation():
    rospy.init_node('cancel_monitor')
    cancel_pub = rospy.Publisher('/robot_cancel', String, queue_size=1)
    
    rospy.loginfo("Cancel Monitor is ready!")
    rospy.loginfo("Press 'c' and Enter to cancel the robot's current task")
    
    while not rospy.is_shutdown():
        try:
            user_input = input().lower()
            if user_input == 'c':
                cancel_pub.publish(String("cancel"))
                rospy.loginfo("Cancellation command sent!")
        except (EOFError, KeyboardInterrupt):
            break

if __name__ == '__main__':
    try:
        monitor_cancellation()
    except rospy.ROSInterruptException:
        pass 