# ROS Packages for Scout Mobile Base

* Start the Gazebo-based simulation (Scout V2)

    ```
    $ roslaunch scout_bringup scout_base_gazebo_sim.launch
    ```

* Start the keyboard tele-op node

    ```
    $ roslaunch scout_bringup scout_teleop_keyboard.launch
    ```

* Star Gazebo-based simulation & NDT localization

    ```
    $ roslaunch scout_bringup scout_ndt_matching.launch
    ```

* Star amcl_localization mode
* crop.world-完整苗圃   crop0.world-需要补苗
    ```
    $ roslaunch scout_bringup scout_sim.launch
    ```
