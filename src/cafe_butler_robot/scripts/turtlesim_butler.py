#!/usr/bin/env python3

import rospy
import math
import threading
import sys
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute, SetPen
from std_msgs.msg import String

class TurtleButler:
    # Define positions
    POSITIONS = {
        'HOME': (1.0, 1.0),
        'KITCHEN': (9.0, 9.0),
        'TABLE1': (3.0, 5.0),
        'TABLE2': (5.0, 5.0),
        'TABLE3': (7.0, 5.0)
    }
    
    def __init__(self):
        rospy.init_node('turtle_butler')
        
        # Wait for turtlesim services
        rospy.wait_for_service('/turtle1/teleport_absolute')
        rospy.wait_for_service('/turtle1/set_pen')
        
        # Create service proxies
        self.teleport = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
        self.set_pen = rospy.ServiceProxy('/turtle1/set_pen', SetPen)
        
        # Publishers and subscribers
        self.cmd_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)
        self.pose_sub = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        self.cancel_sub = rospy.Subscriber('/robot_cancel', String, self.cancel_callback)
        
        # State variables
        self.current_pose = None
        self.confirmation_received = False
        self.task_cancelled = False
        self.moving_to_table = False
        self.input_lock = threading.Lock()
        
        # Initialize position
        self.teleport_home()
        self.draw_markers()
        rospy.loginfo("Turtle Butler is ready to serve!")
        rospy.loginfo("Press Enter to confirm when robot reaches a location")
        
        # Start the main service loop
        self.serve_tables()
    
    def stop_movement(self):
        """Stop the turtle's movement"""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
    
    def cancel_callback(self, msg):
        """Handle cancellation messages"""
        if msg.data == "cancel":
            with self.input_lock:
                self.task_cancelled = True
    
    def get_confirmation(self):
        """Get user confirmation input"""
        try:
            input()  # Wait for Enter key
            return True
        except (EOFError, KeyboardInterrupt):
            return False
    
    def move_to_position(self, target_x, target_y, check_cancel=True):
        """Move turtle to target position with cancellation support"""
        if self.current_pose is None:
            return False, False
        
        if check_cancel:
            with self.input_lock:
                self.task_cancelled = False
        
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.current_pose is None:
                rate.sleep()
                continue
            
            # Check for cancellation
            if check_cancel:
                with self.input_lock:
                    if self.task_cancelled:
                        self.stop_movement()
                        # Make a U-turn by rotating 180 degrees
                        rospy.loginfo("\nTask cancelled! Making U-turn...")
                        current_angle = self.current_pose.theta
                        target_angle = current_angle + math.pi
                        
                        # Normalize target angle
                        while target_angle > math.pi:
                            target_angle -= 2 * math.pi
                        while target_angle < -math.pi:
                            target_angle += 2 * math.pi
                        
                        # Rotate to face opposite direction
                        while abs(self.current_pose.theta - target_angle) > 0.1 and not rospy.is_shutdown():
                            cmd = Twist()
                            cmd.angular.z = 1.0
                            self.cmd_vel_pub.publish(cmd)
                            rate.sleep()
                        
                        self.stop_movement()
                        return False, True
            
            dx = target_x - self.current_pose.x
            dy = target_y - self.current_pose.y
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance < 0.1:
                self.stop_movement()
                return True, False
            
            # Calculate target angle
            target_angle = math.atan2(dy, dx)
            angle_diff = target_angle - self.current_pose.theta
            
            # Normalize angle
            while angle_diff > math.pi: angle_diff -= 2*math.pi
            while angle_diff < -math.pi: angle_diff += 2*math.pi
            
            # Create movement command
            cmd = Twist()
            if abs(angle_diff) > 0.1:
                cmd.angular.z = 0.5 if angle_diff > 0 else -0.5
            else:
                cmd.linear.x = min(0.5, distance)
                cmd.angular.z = 0.0
            
            self.cmd_vel_pub.publish(cmd)
            rate.sleep()
        
        return False, False
    
    def wait_for_confirmation(self, location):
        """Wait for user confirmation with timeout"""
        rospy.loginfo(f"\nWaiting for customer at {location}...")
        rospy.loginfo("Press Enter if food is accepted, or wait for timeout to return food")
        
        # Create a thread for getting input
        self.confirmation_received = False
        input_thread = threading.Thread(target=self._get_input)
        input_thread.daemon = True
        input_thread.start()
        
        timeout = 10.0  # 10 seconds timeout
        start_time = rospy.Time.now()
        
        while (rospy.Time.now() - start_time).to_sec() < timeout:
            if rospy.is_shutdown():
                return False
            
            with self.input_lock:
                if self.confirmation_received:
                    rospy.loginfo(f"Customer accepted food at {location}")
                    return True
            
            rospy.sleep(0.1)
        
        rospy.logwarn(f"No response at {location} after {timeout} seconds")
        return False
    
    def _get_input(self):
        """Helper method to get input in a thread"""
        try:
            sys.stdin.readline()
            with self.input_lock:
                self.confirmation_received = True
        except:
            pass
    
    def teleport_home(self):
        """Teleport the turtle to home position"""
        home_x, home_y = self.POSITIONS['HOME']
        self.teleport(home_x, home_y, 0)
        rospy.sleep(0.5)
    
    def draw_circle(self, x, y, radius=0.2):
        """Draw a circle at the specified position"""
        current_x = self.current_pose.x if self.current_pose else x
        current_y = self.current_pose.y if self.current_pose else y
        current_theta = self.current_pose.theta if self.current_pose else 0
        
        self.teleport(x, y, 0)
        self.set_pen(255, 0, 0, 2, 0)  # Red, thin line
        
        steps = 20
        for i in range(steps + 1):
            angle = 2 * math.pi * i / steps
            self.teleport(
                x + radius * math.cos(angle),
                y + radius * math.sin(angle),
                angle
            )
        
        self.set_pen(255, 255, 255, 1, 1)  # Turn off pen
        self.teleport(current_x, current_y, current_theta)
    
    def draw_markers(self):
        """Draw markers for all positions"""
        for name, pos in self.POSITIONS.items():
            self.draw_circle(pos[0], pos[1])
            rospy.loginfo(f"Marked {name} at position {pos}")
    
    def pose_callback(self, pose):
        self.current_pose = pose
    
    def return_to_home_via_kitchen(self):
        """Return to home position via kitchen"""
        rospy.loginfo("\nReturning to kitchen first...")
        self.move_to_position(*self.POSITIONS['KITCHEN'])
        rospy.loginfo("Moving to home...")
        self.move_to_position(*self.POSITIONS['HOME'])
    
    def handle_cancellation(self, moving_to_table=False):
        """Handle task cancellation based on current movement state"""
        if moving_to_table:
            rospy.loginfo("Cancellation while moving to table, returning via kitchen...")
            self.move_to_position(*self.POSITIONS['KITCHEN'])
            self.move_to_position(*self.POSITIONS['HOME'])
        else:
            rospy.loginfo("Cancellation while moving to kitchen, returning home...")
            self.move_to_position(*self.POSITIONS['HOME'])
    
    def serve_tables(self):
        """Main service loop"""
        while not rospy.is_shutdown():
            try:
                # Get table number
                table_input = input("\nEnter table number (1-3) or 'q' to quit: ")
                
                if table_input.lower() == 'q':
                    break
                
                try:
                    table_num = int(table_input)
                    if 1 <= table_num <= 3:
                        # Move to kitchen
                        rospy.loginfo("\nMoving to kitchen to pick up food...")
                        reached, cancelled = self.move_to_position(*self.POSITIONS['KITCHEN'])
                        
                        if cancelled:
                            rospy.loginfo("Cancellation while moving to kitchen, returning home...")
                            self.move_to_position(*self.POSITIONS['HOME'], check_cancel=False)
                            continue
                        
                        # Wait for kitchen staff to prepare food
                        rospy.loginfo("\nWaiting for kitchen staff to prepare food...")
                        rospy.loginfo("Press Enter when food is ready, or wait for timeout")
                        if not self.wait_for_confirmation('KITCHEN'):
                            rospy.logwarn("Kitchen staff not ready, returning home")
                            self.move_to_position(*self.POSITIONS['HOME'], check_cancel=False)
                            continue
                        
                        # Move to table
                        rospy.loginfo(f"\nFood ready! Moving to table {table_num}...")
                        self.moving_to_table = True  # Set flag for table movement
                        reached, cancelled = self.move_to_position(*self.POSITIONS[f'TABLE{table_num}'])
                        self.moving_to_table = False  # Reset flag
                        
                        if cancelled:
                            rospy.loginfo("Cancellation while moving to table, returning food to kitchen...")
                            # Return to kitchen after U-turn
                            self.move_to_position(*self.POSITIONS['KITCHEN'], check_cancel=False)
                            rospy.loginfo("Food returned to kitchen, going home...")
                            self.move_to_position(*self.POSITIONS['HOME'], check_cancel=False)
                            continue
                        
                        # Wait for table confirmation
                        rospy.loginfo(f"\nWaiting for customer at table {table_num}...")
                        rospy.loginfo("Press Enter if food is accepted, or wait for timeout to return food")
                        if self.wait_for_confirmation(f'TABLE{table_num}'):
                            # Food was accepted, return directly home
                            rospy.loginfo("\nFood delivered successfully, returning home...")
                            self.move_to_position(*self.POSITIONS['HOME'], check_cancel=False)
                            rospy.loginfo("Delivery completed!")
                        else:
                            # No one took the food, return it to kitchen
                            rospy.logwarn(f"No one at table {table_num}, returning food to kitchen...")
                            self.move_to_position(*self.POSITIONS['KITCHEN'], check_cancel=False)
                            rospy.loginfo("Food returned to kitchen, going home...")
                            self.move_to_position(*self.POSITIONS['HOME'], check_cancel=False)
                            
                    else:
                        rospy.logwarn("Please enter a valid table number (1-3)")
                except ValueError:
                    rospy.logwarn("Please enter a valid number")
                    
            except (rospy.ROSInterruptException, KeyboardInterrupt):
                rospy.loginfo("\nShutting down...")
                break

if __name__ == '__main__':
    try:
        butler = TurtleButler()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 