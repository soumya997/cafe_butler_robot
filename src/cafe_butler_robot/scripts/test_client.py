#!/usr/bin/env python3

import rospy
import actionlib
from cafe_butler_robot.msg import DeliveryTaskAction, DeliveryTaskGoal

def delivery_feedback_cb(feedback):
    """Callback function for delivery feedback."""
    rospy.loginfo(f"Current State: {feedback.current_state}")
    rospy.loginfo(f"Current Table: {feedback.current_table}")
    rospy.loginfo(f"Progress: {feedback.progress}%")

def test_single_delivery():
    """Test single table delivery."""
    client = actionlib.SimpleActionClient('delivery_task', DeliveryTaskAction)
    client.wait_for_server()

    goal = DeliveryTaskGoal()
    goal.table_numbers = [1]
    goal.is_multi_order = False

    rospy.loginfo("Sending single delivery goal...")
    client.send_goal(goal, feedback_cb=delivery_feedback_cb)
    
    client.wait_for_result()
    result = client.get_result()
    
    if result.success:
        rospy.loginfo("Single delivery completed successfully")
    else:
        rospy.logwarn(f"Single delivery failed: {result.message}")
        if result.failed_tables:
            rospy.logwarn(f"Failed tables: {result.failed_tables}")

def test_multi_delivery():
    """Test multiple table delivery."""
    client = actionlib.SimpleActionClient('delivery_task', DeliveryTaskAction)
    client.wait_for_server()

    goal = DeliveryTaskGoal()
    goal.table_numbers = [1, 2, 3]
    goal.is_multi_order = True

    rospy.loginfo("Sending multi-delivery goal...")
    client.send_goal(goal, feedback_cb=delivery_feedback_cb)
    
    client.wait_for_result()
    result = client.get_result()
    
    if result.success:
        rospy.loginfo("Multi-delivery completed successfully")
    else:
        rospy.logwarn(f"Multi-delivery failed: {result.message}")
        if result.failed_tables:
            rospy.logwarn(f"Failed tables: {result.failed_tables}")

def test_cancelled_delivery():
    """Test cancellation of delivery."""
    client = actionlib.SimpleActionClient('delivery_task', DeliveryTaskAction)
    client.wait_for_server()

    goal = DeliveryTaskGoal()
    goal.table_numbers = [1, 2, 3]
    goal.is_multi_order = True

    rospy.loginfo("Sending goal that will be cancelled...")
    client.send_goal(goal, feedback_cb=delivery_feedback_cb)
    
    rospy.sleep(2.0) 
    client.cancel_goal()
    rospy.loginfo("Goal cancelled")
    
    client.wait_for_result()
    result = client.get_result()
    
    if result:
        rospy.loginfo(f"Cancellation result: {result.message}")
        if result.failed_tables:
            rospy.loginfo(f"Unserved tables: {result.failed_tables}")

if __name__ == "__main__":
    try:
        rospy.init_node('butler_robot_test_client')
        
        test_single_delivery()
        rospy.sleep(2.0) 
        
        test_multi_delivery()
        rospy.sleep(2.0)  
        
        test_cancelled_delivery()
        
    except rospy.ROSInterruptException:
        pass 