# robot_pushing_cart
Aiming at develop safely moving robot while pushing a cart
# Project is under-developing

# Objectives
* Sensing the cart and the environment
* Motion planning for COB4 robot and the cart
* Control of the robot and the cart along desired path

# 1. Sensing the cart and the environment
1. "Cart State Server" which provides states (pose, pivot) of the cart related to the robot and the environment. The data are collected through laser scan
2. "Arm Control" which guilds COB arms to grasp the cart and hold it tightly

# 2. Motion planning for COB4 robot and the cart
1. "Navigation stack" of COB4
2. How to eleminate the cart laser scanned, which could give unwanted stop to COB4?
3. SBPL package?

# 3. Control of the robot and the cart along desired path
1. COB twist control
