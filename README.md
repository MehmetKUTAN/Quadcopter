It allows the drones to form the formation and move on the navigation path.
--------------------------------------------------------------------


--------------------------------------------------------------------
setup for project;
1. Clone the project file to the desktop
2. Create "src" folder inside the project file
3.Move all files inside Quadcopter into "src" folder
4.Clone geographic-info files into "src" file
5.Clone unique _ identifier files inside "src" file
6.do catkin_make
7.open a new bash and type the ros code
 "roslaunch hector_Astar hector_formation_6_quads.launch " 
 and your gazebo environment will open.
8. open a new bash2 and type the ros code. open a new bash and type the ros code
  a) "source devel/setup.bash"
  b) "rosservice call /quad1/enable_motors "enable:true" "
  c) "rosservice call /quad2/enable_motors "enable:true" "
  d) "rosservice call /quad3/enable_motors "enable:true" "
  e) "rosservice call /quad4/enable_motors "enable:true" "
  f) "rosservice call /quad5/enable_motors "enable:true" "
  g) "rosservice call /quad6/enable_motors "enable:true" "
  then
  "roslaunch hector_Astar hector_formation_6_quads_2.launch"

 9.your project will work 
 --------------------------------------------------------------------
 resources:
 1.https://github.com/ros-geographic-info/unique_identifier/
 
 2.https://github.com/ros-geographic-info/geographic_info
 
--------------------------------------------------------------------
requirements:

ros-noetic -> http://wiki.ros.org/noetic

gazebo -> http://gazebosim.org/download
