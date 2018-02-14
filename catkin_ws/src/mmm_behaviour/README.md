# mmm_behaviour
(watch out for the British spelling, I don't know why I did that)

This is the catkin package for the jsk semi demo.
# to use
On raspberry pi,
```
$ roslaunch mmm_behaviour robot.launch
```
This launches
* `mmm_control robot.launch`(which turns on Arduino communication, RasPi camera, audio, and loads the robot URDF.)
* `image_transport republish`(which listens to /raspicam_node/image/compressed and outputs /raspicam_node/image/image_raw. This is required because AR_track_alvar only listens to raw images. cf:[image_transport](http://wiki.ros.org/image_transport#Library_ROS_API), [node tag in launch files](http://wiki.ros.org/roslaunch/XML/node))
* the AR_track_alvar node(that recognizes AR markers)
* the kxr_hand.py script(which creates the kxr_hand frame, which is positioned relative to the ar marker. cf: [Adding a tf frame](http://wiki.ros.org/tf/Tutorials/Adding%20a%20frame%20%28Python%29))

# structure
## what it does(high-level)
Sees a KXR robot, approaches it, then grabs the thing in KXR's hand, then stores it in MMM's basket.

## what it does(middle-level)
### while the KXR robot can't be seen
if it can't recognize KXR(the AR marker on its chest, to be precise), it wanders around the room looking around.
### when it's found
Once it's located, it approaches KXR based on an algorithm similar what was described in the subsumption architecture robot soccer paper. The camera on the arm is locked on to the AR marker.
### when it's close enough
It opens its hand(which also makes it stop), reaches for KXR's hand(which can be calculated using the relative displacement between AR marker and hand), and closes hand.
### when it closes hand
This peculiar condition is inspired by Herbert the robot. When it closes its hand, The arm is retracted to place what it's holding, into the basket.

## what it does (low-level)
It is based on the subsumption architecture.
### arm
#### default(lowest level)
Pose is "look_forward" but the first servo is rotated around.

Hand is relaxed
#### look(second level)
##### trigger: targetRecognized()
when a marker is detected. This is done by :listening to /visualization_marker, and when it is called, updates the variable markerLastReceived with the current time. When targetRecognized() is called, it compares current time with markerLastReceived, and when it hasn't passed 0.3 seconds after last marker receive, returns True.
##### behaviour

Hand is relaxed.
#### grab(likewise)
##### trigger: targetGrabbable()
first, checks with targetRecognized() to see if target is currently seen. Then, uses the tf listener(cf:[Writing a tf listener (Python)](http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20%28Python%29)) to get the transform between the /base frame and /kxr_hand frame. If xy distance is within 0.15m, returns true.
##### behaviour

Hand is open, reach, hand is closed
#### store
##### trigger: handClosed()
get current joint values of the "gripper" move group(which has just one joint) (cf: [Move Groupt Python tutorial](http://docs.ros.org/hydro/api/pr2_moveit_tutorials/html/planning/scripts/doc/move_group_python_interface_tutorial.html)- "Planning to a joint-space goal"),
##### behaviour
Hand is closed, move hand, hand is open, wait one second, hand is default
### hand
#### default
If not within range of ??~??, change to ??

If it is, keep it relaxed
#### open
##### trigger: targetGrabbable()
##### behaviour
opens hand wide. (relax afterwards)
#### closed
##### trigger:

### wheels
#### default
wander around

#### clockwise turn
##### trigger: ???()
uses the [transformations.py library](https://github.com/ros/geometry/blob/hydro-devel/tf/src/tf/transformations.py) available within tf, to compute euler angles, to see how much angle there is between robot and target

# some more links
* no matter how many times I read this I'll never remember how to write it http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
* source code for moveit_commander https://github.com/ros-planning/moveit/blob/kinetic-devel/moveit_commander/src/moveit_commander/move_group.py
* MoveIt python Tutorial http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/pr2_tutorials/planning/scripts/doc/move_group_python_interface_tutorial.html
