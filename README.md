# visualization_brubotics
Visualization developed by the summer 2021 BruBotics interns.

<p align="center">
  <img width="485" height="424" src="https://github.com/mrs-brubotics/visualization_brubotics/blob/main/.fig/derg5.png">
</p>

This visualization only works with the [two_drones_D-ERG simulation](https://github.com/mrs-brubotics/testing_brubotics/tree/master/tmux_scripts/bryan/two_drones_D-ERG).

Be sure to have the `enable_visualization` variable set to `true` in the [dergbryan_tracker.yaml](https://github.com/mrs-brubotics/trackers_brubotics/blob/master/config/default/dergbryan_tracker.yaml) and an updated ```.yml``` file for the rviz part.
It must look like this:
```
  - rviz:
      layout: tiled
      panes:
        - waitForControl; roslaunch testing_brubotics rviz_brubotics.launch name:=two_drones_derg
        - waitForControl; roslaunch testing_brubotics tf_connector_avoidance.launch 
        - waitForControl; export UAV_NAME=uav1; roslaunch mrs_rviz_plugins load_robot.launch
        - waitForControl; export UAV_NAME=uav2; roslaunch mrs_rviz_plugins load_robot.launch
        - waitForControl; roslaunch visualization_brubotics visual.launch
```
