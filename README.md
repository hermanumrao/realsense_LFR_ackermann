# realsense_LFR_ackermann
This is a package for a line following robot that uses a Realsesnse D435i camera. It is capable of detecting lines on the floor and following them. it even stops on spotting a red area for 5s. This was developed for old warehouse and factories which need automation but  aren't  able to due to their old hardware.

The robot is designed to move around such factories using black take markings on the floor. The user can configure the markings to be of certain width and lenght, this is based on the requirement and floor. Also where ever the user wants the robot to stop they can place a red marker. This will help the robot to stop for a few seconds and then continue to move.

This entire project is based out on ROS2 and the previously developed autonomy stack of mine. 
It is still under validation and testing phase so any for of feature requests, errors or recomendations are appreciated.
