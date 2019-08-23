# Cart-pushing navigation package for Care-o-bot 4
This cob_pushing repository is developed to be a part of cart-pushing navigation and following task in [Intelligent Robotics Group](http://irobotics.aalto.fi). The cob_pushing is aimed to drive the Care-o-bot 4 to desire location in know map while the robot is holding a cart in its hands and keep the cart safe. In this work, I used [SBPL_lattice_planner](http://wiki.ros.org/sbpl_lattice_planner) and [TEB_local_planner](http://wiki.ros.org/teb_local_planner) to generate the trajectory of the robot. The installation of those packages is presented below.

**Dependancies**
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
**Installation**
```terminal
cd {your-workspace}/src
git clone https://github.com/ToanLe147/cob_pushing.git 
```
