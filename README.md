# Trajectory_tracking
Repo for trajectory tracking on a drone.

Upload this package on your drone's ubuntu system with ROS2 Foxy.

## Instructions to run this package

- Create a workspace 'trajectory_ws/src' on drone's system.

- Clone all the packages under 'trajectory_ws/src'

- CLI commands:
    ```
    cd ~/trajectory_ws/
    ```
    ```
    colcon build
    ```
    ```
    source install/setup.bash
    ```
    ```
    ros2 run px4_ros_com offboard_control
    ```