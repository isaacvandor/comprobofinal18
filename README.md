# comprobofinal18
A Repository for our Comprobo 2018 Final Project

For an explanation of PINBot and what it does, see ![our website](https://sites.google.com/view/pinbot/home)

## Run the Code Yourself:
1. clone this git repo to your `~/catkin/src` folder
2. `roslaunch neat_node bringup.launch host:=192.168.xx.xx`
3. cd to the main pinbot folder and run `python astar.py` to create an initial path through the map
4. Run `rosrun pinbot navstar.py` to output velocity commands for the robot
