Modified primitive motions
=============================

There is pre-defined primitive motions for Care-o-bot 4 in [*includes/mprim*](https://github.com/ToanLe147/cob_pushing/tree/master/launch/includes/mprim). Not all of them work fine with the navigation system, some suggestions are presented below. Besides, generating new primitive motion set is available, following this [instruction](http://sbpl.net/node/52) and noticing the resolution of used map cause that is common reason for craeting an unreadable primitive set.

# Suggestions
1. *cob_edited.mprim* - movement is combination of short/medium straight line even in turning action. 
2. *cob_unicycle_new.mprim* - movement is combination of short/medium straight line and in-place rotation. Turning action is smoothier.
3. *full_prim_motion.mprim* - forward, backward, turn-in-place, forward-turn-arcs, backward-turn-arcs, strafing left/right and diagonal move. 

# Using other primitive set
The primitive set could be updated through rosparam server. 
```terminal
rosparam set /move_base_node/SBPLLatticePlanner/primitive_filename {primitive motion file (*.mprim) directory}
```
