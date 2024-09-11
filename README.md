# Stage Navigation - Bug 0  adapted - ROS2

### In this work, a small adaptation of the Bug 0 algorithm was implemented in ROS2 to hit two predetermined targets. 

To run the simulation it is necessary to have the Stage ROS2 simulator installed, available at: [stage_ros2](https://github.com/tuw-robotics/stage_ros2)

Run:

```bash
ros2 launch stage_ros2 stage.launch.py world:=cave enforce_prefixes:=false one_tf_tree:=true
```

And in another terminal run:

```bash
ros2 run stage_navigation stage_navigation_node
```

You can see how the algorithm works here: (https://youtu.be/ywFFFz1vUEI)


![](https://github.com/Andressa-Silva/stage_navigation/blob/main/stage.gif)