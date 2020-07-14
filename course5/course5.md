# Exercise Section 5

- Implement [a service](husky_highlevel_controller/src/HuskyHighlevelController.cpp#L114) to start/stop the robot
- Call service
  ```sh
  roslaunch husky_highlevel_controller husky_controller.launch
  ```
  ```sh
  rosservice call /start_stop 0 # stop robot
  rosservice call /start_stop 1 # start robot
  ```
