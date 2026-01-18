## How to launch 
1. Launch the sim 
2. Launch the program


```bash 
cd ros_space
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run object_detection detection_node

```

In another terminal 
```bash 
cd ros_space
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run object_detection decision_node
```