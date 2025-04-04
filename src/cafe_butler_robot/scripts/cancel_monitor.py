#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def monitor_cancellation():
    rospy.init_node('cancel_monitor', anonymous=True)
    cancel_pub = rospy.Publisher('/robot_cancel', String, queue_size=10)
    rate = rospy.Rate(10)  
    
    rospy.loginfo("Cancel Monitor is ready!")
    rospy.loginfo("Press 'c' and Enter to cancel the robot's current task")
    # Publish multiple times to ensure message is received
    while not rospy.is_shutdown():
        try:
            user_input = input().lower().strip()
            if user_input == 'c':
                msg = String()
                msg.data = "cancel"
                
                for _ in range(5):
                    cancel_pub.publish(msg)
                    rate.sleep()
                    # print("######################")
                    # print(msg)
                rospy.loginfo("Cancellation command sent!")
        except (EOFError, KeyboardInterrupt):
            break

if __name__ == '__main__':
    try:
        monitor_cancellation()
    except rospy.ROSInterruptException:
        pass 