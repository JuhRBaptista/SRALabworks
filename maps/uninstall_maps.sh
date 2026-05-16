#!/bin/bash
#
# uninstall SRA 2023 maps (v05)
#
# versions:  
# v01 - initial version (basic uninstall)
# v02 - remove extra maps + objects
# v03 - compliant with install_maps.sh (v03)
# v04 - added 3 new maps (rmap, xmap, ymap)
# v05 - added cube and cylinder objects  
 
# exit on any command error
set -e 

echo "---uninstalling SRA 2023 maps---"

# remove 3D maps
rm -rf ~/.gazebo/models/bxmap
rm -rf ~/.gazebo/models/csqmap
rm -rf ~/.gazebo/models/fmap
rm -rf ~/.gazebo/models/lcmap
#rm -rf ~/.gazebo/models/mvmap
rm -rf ~/.gazebo/models/rmap
rm -rf ~/.gazebo/models/scmap
rm -rf ~/.gazebo/models/stmap
rm -rf ~/.gazebo/models/umap
rm -rf ~/.gazebo/models/xmap
rm -rf ~/.gazebo/models/ymap

# remove 3D objects
rm -rf ~/.gazebo/models/sra_sign
rm -rf ~/.gazebo/models/box
rm -rf ~/.gazebo/models/cube
rm -rf ~/.gazebo/models/cylinder

# remove launch files
rm -v /opt/ros/melodic/share/turtlebot3_gazebo/launch/tbot3_bxmap.launch
rm -v /opt/ros/melodic/share/turtlebot3_gazebo/launch/tbot3_csqmap.launch
rm -v /opt/ros/melodic/share/turtlebot3_gazebo/launch/tbot3_fmap.launch
rm -v /opt/ros/melodic/share/turtlebot3_gazebo/launch/tbot3_lcmap.launch
#rm -v /opt/ros/melodic/share/turtlebot3_gazebo/launch/tbot3_mvmap.launch
rm -v /opt/ros/melodic/share/turtlebot3_gazebo/launch/tbot3_rmap.launch
rm -v /opt/ros/melodic/share/turtlebot3_gazebo/launch/tbot3_scmap.launch
rm -v /opt/ros/melodic/share/turtlebot3_gazebo/launch/tbot3_stmap.launch
rm -v /opt/ros/melodic/share/turtlebot3_gazebo/launch/tbot3_umap.launch
rm -v /opt/ros/melodic/share/turtlebot3_gazebo/launch/tbot3_xmap.launch
rm -v /opt/ros/melodic/share/turtlebot3_gazebo/launch/tbot3_ymap.launch

# remove world files
rm -v /opt/ros/melodic/share/turtlebot3_gazebo/worlds/tbot3_bxmap.world
rm -v /opt/ros/melodic/share/turtlebot3_gazebo/worlds/tbot3_csqmap.world
rm -v /opt/ros/melodic/share/turtlebot3_gazebo/worlds/tbot3_fmap.world
rm -v /opt/ros/melodic/share/turtlebot3_gazebo/worlds/tbot3_lcmap.world
#rm -v /opt/ros/melodic/share/turtlebot3_gazebo/worlds/tbot3_mvmap.world
rm -v /opt/ros/melodic/share/turtlebot3_gazebo/worlds/tbot3_rmap.world
rm -v /opt/ros/melodic/share/turtlebot3_gazebo/worlds/tbot3_scmap.world
rm -v /opt/ros/melodic/share/turtlebot3_gazebo/worlds/tbot3_stmap.world
rm -v /opt/ros/melodic/share/turtlebot3_gazebo/worlds/tbot3_umap.world
rm -v /opt/ros/melodic/share/turtlebot3_gazebo/worlds/tbot3_xmap.world
rm -v /opt/ros/melodic/share/turtlebot3_gazebo/worlds/tbot3_ymap.world

# remove run scripts
rm -v ~/start-gazebo-bxmap.sh
rm -v ~/start-gazebo-csqmap.sh
rm -v ~/start-gazebo-fmap.sh
rm -v ~/start-gazebo-lcmap.sh
#rm -v ~/start-gazebo-mvmap.sh
rm -v ~/start-gazebo-rmap.sh
rm -v ~/start-gazebo-scmap.sh
rm -v ~/start-gazebo-stmap.sh
rm -v ~/start-gazebo-umap.sh
rm -v ~/start-gazebo-xmap.sh
rm -v ~/start-gazebo-ymap.sh

echo "---all maps removed---"


