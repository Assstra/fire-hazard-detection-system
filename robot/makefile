.PHONY: all

start:
	roslaunch oit_navigation_minibot_light_01 stage_navigation.launch map_name:=map_20250604_140633

run:
	rosrun fire-hazard-detection-system node.py

debug:
	rosrun fire-hazard-detection-system node.py --debug

topic:
	    @if [ -z "$(X)" ] || [ -z "$(Y)" ]; then \
        echo "Usage: make topic X=<x> Y=<y>"; \
    else \
        rostopic pub -1 /alert geometry_msgs/Pose -- "[$(X), $(Y), 0]" "[0, 0, 0, 1]"; \
    fi

make:
	cd ../../.. && catkin_make