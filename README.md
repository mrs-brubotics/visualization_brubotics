# visualization_brubotics
Visualization developed by the summer 2021 BruBotics interns.

![](https://github.com/mrs-brubotics/visualization_brubotics/blob/main/.fig/rviz_window.png)

This visualization only works with the [2_two_drones_D-ERG simulation](https://github.com/mrs-brubotics/testing_brubotics/tree/master/tmux_scripts/bryan/2_two_drones_D-ERG).

Be sure to have an updated ```.yaml``` file for the rviz part.
It must look like this:
```
- rviz:
    layout: tiled
    panes:
      - waitForControl; roslaunch visualization_brubotics rviz.launch name:=avoidance_test
      - waitForControl; roslaunch visualization_brubotics load_robot.launch
```
