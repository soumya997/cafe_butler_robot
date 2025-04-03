# Cafe Butler Robot
 This is a simple ROS1 based butler robot that pretends to deliver food in a cafe. It's pretty basic right now - just moving between states and publishing messages.

## Working

The robot goes through a cycle of states, kind of like a really simple delivery person:

1. Starts at home (probably charging or just chilling)
2. Goes to the kitchen (takes 5 seconds to get there)
3. Hangs out at the kitchen for 2 seconds (pretending to pick up food)
4. Heads to a table (another 5-second journey)
5. Stays at the table for 2 seconds (dropping off the food)
6. Goes back home (5 more seconds)
7. Takes a quick 5-second break
8. Rinse and repeat!

## State Changes (The Fun Part!)

Here's how our robot changes its mood (states):
```
AT_HOME -> "Alright, time to work!"
MOVING_TO_KITCHEN -> "Walking to kitchen... beep boop"
AT_KITCHEN -> "Yo chef, got any orders ready?"
MOVING_TO_TABLE -> "Coming through! Hot food!"
AT_TABLE -> "Here's your order! Enjoy!"
RETURNING_HOME -> "Heading back to my charging spot!"
```

Each state change is just simulated with a timer right now this is what i could do in a short time. In a real robot, this would be WAY more complex with actual movement, sensors, and probably a lot of bumping into things during testing 

## Requirements

- ROS Noetic
- Python 3.x
- Just two ROS packages:
  - rospy
  - std_msgs

## Getting Started

1. Clone this bad boy:
```bash
cd ~/catkin_ws/src
git clone https://github.com/soumya997/cafe_butler_robot.git
```

2. Build it:
```bash
cd ~/catkin_ws
catkin_make
```

3. Source it (don't forget this or nothing will work!):
```bash
source ~/catkin_ws/devel/setup.bash
```

## Running the Robot

You need three terminal windows (I know, I know, but that's ROS for you):

1. run the roscore
```bash
roscore
```

2. run the actual robot controller node, that moves based on the state changes
```bash
rosrun cafe_butler_robot butler_robot_controller.py
```

3. run the test client node to listen to the state changes and print them:
```bash
rosrun cafe_butler_robot test_client.py
```




## Future Ideas (aka "Things We Should Probably Add")

- Actually detect when we reach places (instead of just waiting 5 seconds)
- Avoid running into things (kind of important)
- Real navigation (instead of just pretending to move)
- Actually pick up and put down food (robot arms, anyone?)
- Make sure we don't spill drinks everywhere
- A proper UI for the kitchen staff
- Emergency stop (for when things go wrong, and they will!)
- add a behavior tree to make it more realistic and handle different tasks

# Actionlib
Actionlib is like a smart message system for complex robot tasks. Instead of just sending simple one way msgs (like "I'm at kitchen"), it handles the whole convo between the kitchen and robot. The kitcen can send a task ("deliver to table 5"), get progress updates ("moving to table, 50% done"), cancel the task if needed ("oops, wrong table!"), and know if it succeded or failed ("delivery complete!" or "couldn't reach table"). It's there to managing long-running tasks that need feedback and control, rather than just simple status updates.  `DeliveryTask.action` file because it defines the structure of the action messages used by actionlib.
