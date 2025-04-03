#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute, SetPen

class PositionMarker:
    def __init__(self):
        rospy.init_node('position_marker')
        
        # Wait for turtlesim services
        rospy.wait_for_service('/turtle1/teleport_absolute')
        rospy.wait_for_service('/turtle1/set_pen')
        
        # Create service proxies
        self.teleport = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
        self.set_pen = rospy.ServiceProxy('/turtle1/set_pen', SetPen)
        
        # Store original position
        self.original_x = 0
        self.original_y = 0
        self.original_theta = 0
        
        # Subscribe to pose to get initial position
        self.pose_sub = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback, queue_size=1)
        
    def pose_callback(self, pose):
        self.original_x = pose.x
        self.original_y = pose.y
        self.original_theta = pose.theta
        self.pose_sub.unregister()  # Only need initial position
        self.draw_markers()
    
    def draw_circle(self, x, y, radius=0.2):
        # Teleport to position
        self.teleport(x, y, 0)
        
        # Set pen properties (R, G, B, width, off)
        self.set_pen(255, 0, 0, 2, 0)  # Red, thin line
        
        # Draw circle
        steps = 20
        for i in range(steps + 1):
            angle = 2 * math.pi * i / steps
            self.teleport(
                x + radius * math.cos(angle),
                y + radius * math.sin(angle),
                angle
            )
    
    def draw_markers(self):
        # Dictionary of positions
        positions = {
            'HOME': (1.0, 1.0),
            'KITCHEN': (9.0, 9.0),
            'TABLE1': (3.0, 5.0),
            'TABLE2': (5.0, 5.0),
            'TABLE3': (7.0, 5.0)
        }
        
        # Draw a circle at each position
        for pos in positions.values():
            self.draw_circle(pos[0], pos[1])
        
        # Turn off pen and return to original position
        self.set_pen(255, 255, 255, 1, 1)  # Turn off pen
        self.teleport(self.original_x, self.original_y, self.original_theta)

if __name__ == '__main__':
    try:
        marker = PositionMarker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 