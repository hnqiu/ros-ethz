# Exercise Section 1

- Setup & run the [Husky simulation](http://wiki.ros.org/husky_gazebo/Tutorials/Simulating%20Husky)
  ```sh
  # install & setup
  sudo apt-get install ros-<distro>-husky-simulator
  export HUSKY_GAZEBO_DESCRIPTION=$(rospack find husky_gazebo)/urdf/description.gazebo.xacro

  # launch simulation
  roslaunch husky_gazebo husky_empty_world.launch
  ```
- Send velocity command
  ```sh
  # show topic message info
  rostopic type /husky_velocity_controller/cmd_vel | rosmsg show
  # publish message
  rostopic pub /husky_velocity_controller/cmd_vel geometry_msgs/Twist -r 1 '[2.0, .0, .0]' '[.0, .0, 1.0]'
  ```
- Use `teleop_twist_keyboard`
  ```sh
  # download & build
  cd ~/git
  git clone https://github.com/ros-teleop/teleop_twist_keyboard.git
  cd ~/catkin_ws/src
  ln -s ~/git/teleop_twist_keyboard/
  cd ..
  catkin build teleop_twist_keyboard
  source devel/setup.bash

  # test
  # launch the husky robot
  roslaunch husky_gazebo husky_empty_world.launch
  # in a new terminal
  rosrun teleop_twist_keyboard teleop_twist_keyboard.py
  # and use the terminal to move the robot
  ```
- Write a [launch file](ex1_5.launch)
  ```sh
  roslaunch ex1_5.launch
  # and use this terminal to navigate the robot
  ```
