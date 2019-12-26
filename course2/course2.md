# Exercise Section 2

- (easy way) Download the package [`husky_highlevel_controller.zip`](https://ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/rsl-dam/ROS2019/husky_highlevel_controller.zip) and unzip it to folder `~/catkin_ws/src/`.

- Create subscriber to topic `/scan` with callback method
  - Check topic `/scan`
    ```sh
    # launch husky_gazebo
    roslaunch husky_gazebo husky_empty_world.launch
  
    # in a new terminal
    rostopic type /scan # get the topic type, which is 'sensor_msgs/LaserScan'
    ```
  - Create callback method
    ```cpp
    void callback(const sensor_msgs::LaserScan &msg);
    ```
  - Create subscriber
    ```cpp
    ros::Subscriber sub = nh.subscribe("/scan", 1, callback);
    ```
- Add [param file](husky_highlevel_controller/config/default.yaml) & create c++ API to load params
  ```cpp
  ros::getParam("param", var);
  ```
- Create [launch file](husky_highlevel_controller/launch/husky_controller.launch) 
  - add node `husky_highlevel_controller`
  - load param file
  - launch `husky_empty_world.launch` with argument `laser_enabled`
  - add `RViz`
- RViz configuration
  - Set _Fixed Frame_ under the __Global Options__ to `odom`
  - Add plugin __LaserScan__ and set the _Topic_ as `/scan`; change the _Size_ to have the best visualization of objects
  - Save the configuration
