# comprobofinal18
A Repository for our Comprobo 2018 Final Project

For an explanation of PINBot and what it does, see ![our website](https://sites.google.com/view/pinbot/home)

## Run the Code Yourself:
### To Just Run the Robot with Pre-Computed points, paths, etc.
1. clone this git repo to your `~/catkin/src` folder
2. `roslaunch neat_node bringup.launch host:=192.168.xx.xx`
3. cd to the main pinbot folder and run `python astar.py` to create an initial path through the map
4. Run `rosrun pinbot navstar.py` to output velocity commands for the robot

### To compute the points, paths, etc yourself:
1. Follow steps 1-2 above
2. Run `rosrun neato_2dnav gmapping_demo.launch` to begin collecting map data
3. Run `map_server map_saver -f /path/to/pinbot/folder/maps` to save map file
4. Open `astar.py` and `navstar.py` to change path to map file to current map file path
5. Run `python astar.py` with the new map file path
6. Run `map_server map_server /path/to/map/file` to load the map into the map_server
7. Run `rosrun pinbot image_to_points.py` to convert the map image to points
8. Run `rosrun pinbot particle_clustering.py` to run the particle clustering algorithm
9. Run `rosrun pinbot find_circle.py` to identify the circles and publish that data
10. Run `rosrun pinbot marker_viz.py` to visualize/confirm the identification code
11. Run `rosrun pinbot navstar.py` to run the A* algorithms and output velocity commands to the robot
12. Sit back, relax, and enjoy while your neato effortlessly threads its way through a mine field to get to a round trashcan
