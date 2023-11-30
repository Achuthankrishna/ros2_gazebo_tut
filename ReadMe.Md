# vacuum_cleaner_bot
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
---

### Dependencies:
- OS : Ubuntu 22.04 
- ROS2 Distro : ROS2 Humble
- Package build type : ```ament_cmake ```
- Package dependencies : ```rclcpp```, ```std_msgs``` 
- ROS2 Humble Installation : [link](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html)

## Essential Packages
- Turtlebot3 Simulation for ros2: [link](https://github.com/ROBOTIS-GIT/turtlebot3/tree/ros2)




### Instructions to build the ROS Package
- Soucrce ROS2 
```
source /opt/tos/humble/setup.bash
```


- clone the repo in the ros2 workspace/src

```
rosdep install -i --from-path src --rosdistro humble -y

colcon build --packages-select ros2_gazebo_tut
```

### Launch without rosbag

```
ros2 launch ros2_gazebo_tut ros2_world.launch.py
```

### Launch with rosbag

```
ros2 launch ros2_gazebo_tut ros2_world_rbag.launch.py
```

- The ros bag file will be saved in the results folder

## RosBag Check
 - Go to the destination
 ```
 cd ./src/ros2_gazebo_tuts/results/Bags
 ```

 To play the ros bag , just type
 ```
 ros2 bag play Bags_0.db3
 ```

 Simultaneously you can open another terminal and hit 
 ```
 ros2 topic list
 ```

 select any topic and just echo it to find the recordings.

### cppcheck
```
cppcheck --enable=all --std=c++17 ./src/*.cpp --suppress=missingIncludeSystem --suppress=unmatchedSuppression --suppress=unusedFunction --suppress=missingInclude --suppress=useInitializationList > results/cppcheck.txt
```

### cpplint
```
 cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order ./src/*.cpp ./include/*.hpp > ./results/cpplint.txt
```