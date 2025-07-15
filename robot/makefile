.PHONY: all

start:
	roslaunch oit_navigation_minibot_light_01 stage_navigation.launch map_name:=map_20250604_140633

run:
	cd ../../.. && catkin_make
	rosrun fire-hazard-detection-system node.py

make:
	cd ../../.. && catkin_make