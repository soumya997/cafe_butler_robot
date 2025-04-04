# Cafe Butler Robot

I added a ROS-based robot butler system using turtlesim for visualization. The robot serves as a waiter, delivering food from the kitchen to different tables in turtle sim window.

<img src="media/Untitled Diagram.png" width="500">

youtube: https://youtu.be/YYtP6SdH0LA


## System Components

### Files and Their Purpose

1. `position_marker.py`: Initializes the turtlesim environment by marking positions for:
   - Home (1,1)
   - Kitchen (9,9)
   - Table 1 (3,5)

   - Table 2 (5,5)
   - Table 3 (7,5)


2. `turtlesim_butler.py`: Main robot control script that:
   - Handles robot movement between positions
   - Manages food delivery tasks
   - Processes user confirmations
   - Implements cancellation behavior

3. `cancel_monitor.py`: Provides a separate interface for task cancellation:
   - Monitors for 'c' key press
   - Publishes cancellation commands to the robot

## How It Works

The system uses turtlesim to visualize a robot butler in a cafe environment. The turtle represents the robot, and red circles mark different locations (home, kitchen, tables). The robot:

1. Starts at the home position
2. Moves to kitchen to collect food
3. Waits for kitchen staff confirmation
4. Delivers to specified tables
5. Waits for customer confirmation at each table
6. Returns home after completing deliveries

### Cancellation Behavior
- If canceled while moving to kitchen: Robot returns directly home
- If canceled while moving to table: Robot returns to kitchen first, then home
- If no confirmation at table: Continues to next table (for multiple orders) or returns via kitchen

## Running the System

Open four terminal windows and run these commands in order:

1. Start ROS core:
```bash
roscore
```

2. Launch turtlesim:
```bash
rosrun turtlesim turtlesim_node
```

3. Start the robot butler:
```bash
rosrun cafe_butler_robot turtlesim_butler.py
```

4. Start the cancellation monitor:
```bash
rosrun cafe_butler_robot cancel_monitor.py
```

## Usage

1. Enter table numbers when prompted (e.g., "1,2,3" or "2,3")
2. Press Enter to confirm when robot reaches kitchen/table
3. Press 'c' in the cancel_monitor terminal to cancel current task
4. Enter 'q' to quit the program
