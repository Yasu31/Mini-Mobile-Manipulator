# Mini Mobile Manipulator
storage for Individual Projects(自主プロ) at UTokyo

## [Progress is documented in the wiki.](https://github.com/Yasu31/fantastic-octo-parakeet/wiki)

## demo
general run-through of robot
[![](img/video1.png)](https://www.youtube.com/watch?v=AJrDMAA22wg)

pick up trash(kind of)
[![](img/video2.png)](https://www.youtube.com/watch?v=ZHt5yPhyLIU)

## documents
### final report for school project(Japanese)
[![report_image](img/report.png)](/project_submissions/mmm_jishupro_report.pdf)

## To move the robot(as of Jan.10)
* make sure to be on the same network as robot(NOT eduroam)
* For VM Ubuntu, set ROS_MASTER_URI to http://(IP of raspi):11311 inside ~/.bashrc, set robot_ip on moveit_planning_execution.launch to (IP of raspi).
* `roslaunch mmm_control robot.launch` on RasPi, which launches roscore, handles communication between Arduino, and audio, and ...
* `roslaunch mmm_moveit_config moveit_planning_execution.launch` which starts MoveIt and RVIZ.
* `rosrun image_view image_view image:=/raspicam_node/image _image_transport:=compressed` to view camera feed


![](https://github.com/Yasu31/fantastic-octo-parakeet/blob/master/img/model.jpg)

careful- the MMM currently sources the  setup file  for the ice-cream catkin_ws!

## To Move the robot (for MayFes)
* make sure to be on same network as robot
* on the computer (not RasPi), set ROS_MASTER_URI to http://(IP of raspi):11311 inside ~/.bashrd, set robot_ip to IP of raspi in MoveIt file as well
* Run `roslaunch mmm_control only_arduino.launch`on the robot and `roslaunch mmm_control camera.launch` on either computer
* If the camera is connected to the robot raspi, running `robot.launch` will launch both previous launch files
* Run `roslaunch mmm_control ar_track.launch` on some computer, to start tracking AR marker
* Run `roslaunch mayfes behavior.launch` to start the actual behavior created for MayFes.
