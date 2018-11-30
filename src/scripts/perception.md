# Neato Perception Stack

## Neato Navigation (Simulator)
roslaunch neato_simulator neato_playground.launch
rosrun map_server map_server `rospack find neato_2dnav`/maps/playground_smaller.yaml
roslaunch neato_2dnav amcl_builtin.launch map_file:=`rospack find neato_2dnav`/maps/playground_smaller.yaml


## Neato Navigation (Onboard)
roslaunch neato_node bringup.launch host:=192.168.xxx.xxx
roslaunch neato_2dnav move_base.launch
roslaunch neato_2dnav gmapping_demo.launch scan_topic:=/stable_scan
roslaunch turtlebot_rviz_launchers view_navigation.launch
