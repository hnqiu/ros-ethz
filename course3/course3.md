# Exercise Section 3

- Edit the [launch file](husky_highlevel_controller/launch/husky_controller.launch)
- Edit the [callback method](husky_highlevel_controller/src/HuskyHighlevelController.cpp#L76) to extract the pillar coordinate with respect to the robot  
  Notes:
  - `angle_min = -135 deg`; `angle_max = 135 deg`
  - seems like the coordinate system of husky is different from the pdf illustration: `y` is to the right
- Add `geometry_msgs` to `CMakeLists.txt` & `package.xml` and create a publisher on `/cmd_vel`
  - create [`setVel()`](husky_highlevel_controller/src/HuskyHighlevelController.cpp#L36) to set Husky velocity
  - create [`DriveHusky()`](husky_highlevel_controller/src/HuskyHighlevelController.cpp#L49) to publish the Twist message to Husky
- Drive Husky towards pillar with a [P controller](husky_highlevel_controller/src/HuskyHighlevelController.cpp#L57)
- Add RobotModel & TF to RViz
- [Visualize pillar](husky_highlevel_controller/src/HuskyHighlevelController.cpp#L64) in RViz
