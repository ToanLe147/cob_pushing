# robot_pushing_cart
Aiming at develop safely moving robot while pushing a cart
# Project is under-developing

# Objectives
* ~~Sensing the cart and the environment~~
* ~~Motion planning for COB4 robot and the cart~~
* ~~Control of the robot and the cart along desired path~~

# * Sensing the cart and the environment
1. ~~"Cart State Server" which provides states (pose, pivot) of the cart related to the robot and the environment. The data are collected through laser scan~~
2. ~~"Arm Control" which guilds COB arms to grasp the cart and hold it tightly~~

# * Motion planning for COB4 robot and the cart
1. ~~"Navigation stack" of COB4~~
2. ~~How to eleminate the cart laser scanned, which could give unwanted stop to COB4?~~
3. ~~SBPL package?~~
4. ~~Global planning for cart pushing alone task~~
5. ~~Local planning for cart pushing and nurse following combined task~~
6. ~~SBPL in following task~~

# * Control of the robot and the cart along desired path
1. ~~COB twist control~~
2. ~~Primitive Motion with/without cart - generated "*.mprim" file~~
3. ~~State recovery - Don't rotate. Prefer using forward/backward~~
4. ~~Arm control for turning cart~~
5. ~~Turning cart to follow planned path~~

# CURRENT STATUS: SMOOTHLY NAVIGATION
# CURRENT TASK: GENERATE PRIMITIVE MOTION FOR TRANSPORTING HEAVY CART
