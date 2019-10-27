Cart-pushing navigation package for Care-o-bot 4
================================================

This cob_pushing repository is developed to be a part of cart-pushing navigation and following task in [Intelligent Robotics Group](http://irobotics.aalto.fi). The cob_pushing is aimed to drive the Care-o-bot 4 to desire location in know map while the robot is holding a cart in its hands and keep the cart safe. In this work, I used [SBPL_lattice_planner](http://wiki.ros.org/sbpl_lattice_planner) and [TEB_local_planner](http://wiki.ros.org/teb_local_planner) to generate the trajectory of the robot. The installation of those packages is presented below.

## Videos
<p align="center">
<a href="http://www.youtube.com/watch?feature=player_embedded&v=J7AZaEtInVc" target="_blank"><img src="http://img.youtube.com/vi/J7AZaEtInVc/0.jpg" alt="Simulation Result" width="450" height="315" border="0" /></a>

<a href="http://www.youtube.com/watch?feature=player_embedded&v=PkO12dNXApk" target="_blank"><img src="http://img.youtube.com/vi/PkO12dNXApk/0.jpg" alt="Real Robot Result" width="450" height="315" border="0" /></a>

## Requirements
* [Care-o-bot 4 package](http://wiki.ros.org/care-o-bot)
* SBPL_lattice_planner
```terminal
sudo apt-get install ros-kinetic-sbpl ros-kinetic-sbpl-lattice-planner
```
* TEB_local_planner
```terminal
sudo apt-get install ros-kinetic-teb-local-planner
```
* Twist recovery (replace default rotation recovery behavior by backward movement recovery)
```terminal
sudo apt-get install ros-kinetic-sbpl ros-kinetic-twist-recovery
```
## Installation
```terminal
cd {your-workspace}/src
git clone https://github.com/ToanLe147/cob_pushing.git 
```
## How to use
* Simulation with Care-o-bot 4 only.
```terminal
roslaunch cob_pushing cob_in_stage.launch
```
* Simulation cart-pushing navigation task
```terminal
roslaunch cob_pushing cob_pushing_in_stage.launch
```
* Real robot with cart attached to its hands
```terminal
export ROS_MASTER_URI=http://cob4-8-b1:11311
export ROBOT=cob4-8
export ROBOT_ENV=empty
roslaunch cob_bringup dashboard.launch
roslaunch cob_pushing cob_pushing_executive.launch
```
[Modify Primitive Motion](https://github.com/ToanLe147/cob_pushing/tree/master/launch)
