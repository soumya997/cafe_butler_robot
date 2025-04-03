#!/usr/bin/env python3

import rospy
import actionlib
from enum import Enum
from cafe_butler_robot.msg import DeliveryTaskAction, DeliveryTaskGoal, DeliveryTaskResult, DeliveryTaskFeedback

class RobotState(Enum):
    AT_HOME = "AT_HOME"
    MOVING_TO_KITCHEN = "MOVING_TO_KITCHEN"
    AT_KITCHEN = "AT_KITCHEN"
    MOVING_TO_TABLE = "MOVING_TO_TABLE"
    AT_TABLE = "AT_TABLE"
    RETURNING_TO_KITCHEN = "RETURNING_TO_KITCHEN"
    RETURNING_HOME = "RETURNING_HOME"

class ButlerRobotController:
    def __init__(self):
        self.node_name = "butler_robot_controller"
        rospy.init_node(self.node_name)
        
        self.action_server = actionlib.SimpleActionServer(
            "delivery_task",
            DeliveryTaskAction,
            execute_cb=self.execute_delivery_task,
            auto_start=False
        )
        
        self.current_state = RobotState.AT_HOME
        self.current_table = 0
        self.tables_to_serve = []
        self.failed_tables = []
        
        self.kitchen_timeout = rospy.get_param("~kitchen_timeout", 30.0)  # seconds
        self.table_timeout = rospy.get_param("~table_timeout", 30.0)  # seconds
        self.movement_time = rospy.get_param("~movement_time", 5.0)  # seconds
        
        self.action_server.start()
        rospy.loginfo(f"{self.node_name} initialized and ready")

    def execute_delivery_task(self, goal):
        """Execute a delivery task for single or multiple tables."""
        self.tables_to_serve = goal.table_numbers
        self.failed_tables = []
        result = DeliveryTaskResult()
        feedback = DeliveryTaskFeedback()
        
        if not self.tables_to_serve:
            result.success = False
            result.message = "No tables specified in the order"
            self.action_server.set_succeeded(result)
            return

        try:
            if not self._move_to_kitchen():
                result.success = False
                result.message = "Failed to get confirmation from kitchen"
                result.failed_tables = self.tables_to_serve
                self.action_server.set_succeeded(result)
                return

            for table in self.tables_to_serve:
                if self.action_server.is_preempt_requested():
                    self._handle_cancellation()
                    break

                self.current_table = table
                feedback.current_table = table
                feedback.current_state = RobotState.MOVING_TO_TABLE.value
                self.action_server.publish_feedback(feedback)

                if not self._deliver_to_table(table):
                    self.failed_tables.append(table)
                    continue

            self._return_to_kitchen()
            self._return_home()

            result.success = len(self.failed_tables) == 0
            result.message = "Delivery completed" if result.success else "Some deliveries failed"
            result.failed_tables = self.failed_tables
            self.action_server.set_succeeded(result)

        except Exception as e:
            rospy.logerr(f"Error during delivery task: {str(e)}")
            result.success = False
            result.message = f"Error during delivery: {str(e)}"
            result.failed_tables = self.tables_to_serve
            self.action_server.set_succeeded(result)

    def _move_to_kitchen(self):
        """Move robot to kitchen and wait for confirmation."""
        self.current_state = RobotState.MOVING_TO_KITCHEN
        rospy.sleep(self.movement_time)  # Simulate movement
        self.current_state = RobotState.AT_KITCHEN
        
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < self.kitchen_timeout:
            if self.action_server.is_preempt_requested():
                return False
            rospy.sleep(0.1)
            
            return True  
            
        return False

    def _deliver_to_table(self, table_number):
        """Deliver order to specified table."""
        self.current_state = RobotState.MOVING_TO_TABLE
        rospy.sleep(self.movement_time)  # Simulate movement
        self.current_state = RobotState.AT_TABLE
        
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < self.table_timeout:
            if self.action_server.is_preempt_requested():
                return False
            rospy.sleep(0.1)
            
            return True  
            
        return False

    def _handle_cancellation(self):
        """Handle task cancellation."""
        if self.current_state in [RobotState.MOVING_TO_TABLE, RobotState.AT_TABLE]:
            self._return_to_kitchen()
        self._return_home()
        
        # add the meesage
        result = DeliveryTaskResult()
        result.success = False
        result.message = "Task cancelled"
        result.failed_tables = self.tables_to_serve[self.tables_to_serve.index(self.current_table):]
        self.action_server.set_preempted(result)

    def _return_to_kitchen(self):
        """Return robot to kitchen."""
        self.current_state = RobotState.RETURNING_TO_KITCHEN
        rospy.sleep(self.movement_time)  
        self.current_state = RobotState.AT_KITCHEN

    def _return_home(self):
        """Return robot to home position."""
        self.current_state = RobotState.RETURNING_HOME
        rospy.sleep(self.movement_time) 
        self.current_state = RobotState.AT_HOME

if __name__ == "__main__":
    try:
        controller = ButlerRobotController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 