#!/usr/bin/env python3

import rospy
import actionlib
from cafe_butler_robot.msg import DeliveryTaskAction, DeliveryTaskGoal

def send_delivery_order(table_number):
    """Send a delivery order for a single table"""
    client = actionlib.SimpleActionClient('delivery_task', DeliveryTaskAction)
    rospy.loginfo("Waiting for server...")
    client.wait_for_server()
    
    goal = DeliveryTaskGoal()
    goal.table_numbers = [table_number]
    goal.is_multi_order = False
    
    rospy.loginfo(f"\nSending order for table {table_number}")
    rospy.loginfo("You will need to confirm at both kitchen and table")
    rospy.loginfo("Press Enter to confirm or wait for 10-second timeout")
    
    # Send the goal
    client.send_goal(
        goal,
        feedback_cb=lambda fb: rospy.loginfo(f"Robot Status: {fb.current_state} (Table {fb.current_table})")
    )
    
    # Wait for result
    client.wait_for_result()
    result = client.get_result()
    
    if result.success:
        rospy.loginfo(f"\nDelivery completed successfully!")
    else:
        rospy.logwarn(f"\nDelivery failed for tables {result.failed_tables}")
        rospy.logwarn(f"Reason: {result.message}")

def main():
    rospy.init_node('test_client')
    
    while not rospy.is_shutdown():
        try:
            # Get table number from user
            table_input = input("\nEnter table number (1-3) or 'q' to quit: ")
            
            if table_input.lower() == 'q':
                break
            
            try:
                table_number = int(table_input)
                if 1 <= table_number <= 3:
                    send_delivery_order(table_number)
                else:
                    rospy.logwarn("Please enter a valid table number (1-3)")
            except ValueError:
                rospy.logwarn("Please enter a valid number")
                
        except (rospy.ROSInterruptException, KeyboardInterrupt):
            break

if __name__ == '__main__':
    main() 