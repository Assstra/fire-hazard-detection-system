.PHONY: all

start:
	roslaunch oit_navigation_minibot_light_01 stage_navigation.launch map_name:=map_20250604_140633

start_dx:
	roslaunch oit_navigation_minibot_light_01 stage_navigation.launch map_name:=map_20250728_134358

run:
	rosrun fire-hazard-detection-system main.py --waypoints waypoints/campus.txt $(ARGS)

run_dx:
	rosrun fire-hazard-detection-system main.py --waypoints waypoints/dx.txt $(ARGS)

debug:
	rosrun fire-hazard-detection-system main.py --waypoints waypoints/campus.txt --debug $(ARGS)

debug_dx:
	rosrun fire-hazard-detection-system main.py --waypoints waypoints/dx.txt --debug $(ARGS)

alert:
	rosrun fire-hazard-detection-system main.py --waypoints waypoints/campus.txt --alert $(ARGS)

alert_dx:
	rosrun fire-hazard-detection-system main.py --waypoints waypoints/dx.txt --alert $(ARGS)

topic:
	    @if [ -z "$(X)" ] || [ -z "$(Y)" ]; then \
        echo "Usage: make topic X=<x> Y=<y>"; \
    else \
        rostopic pub -1 /alert geometry_msgs/Pose -- "[$(X), $(Y), 0]" "[0, 0, 0, 1]"; \
    fi

make:
	cd ../../.. && catkin_make