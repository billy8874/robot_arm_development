# Applications &middot; [![ROS noetic](https://img.shields.io/badge/ROS-noetic-blue)](http://wiki.ros.org/noetic) [![Moveit 1 noetic](https://img.shields.io/badge/Moveit%201-noetic-blue)](https://ros-planning.github.io/moveit_tutorials/)

Move group C++ interface allows us to use API to create applications, such as collision detection, obstacle avoidance, and object attachment (todo). [Move Group C++ Interface](https://ros-planning.github.io/moveit_tutorials/doc/move_group_interface/move_group_interface_tutorial.html#)

## Launch
To use move group C++, keep moveit on and open another window to run the following command:
```
rosrun applications avoid_obstacle
```
Then click the `next` button on Rviz visualization toolbar to implement applications on the robot.

## Code
### Obstacle Avoidance
```cpp
// Define a box to add to the world.
shape_msgs::SolidPrimitive primitive;
primitive.type = primitive.BOX;
primitive.dimensions.resize(3);
primitive.dimensions[primitive.BOX_X] = 0.1;
primitive.dimensions[primitive.BOX_Y] = 1.5;
primitive.dimensions[primitive.BOX_Z] = 0.5;

// Define a pose for the box (specified relative to frame_id)
geometry_msgs::Pose box_pose;
box_pose.orientation.w = 1.0;
box_pose.position.x = 0.5;
box_pose.position.y = 0.0;
box_pose.position.z = 0.25;

// Add a collision object to the world
collision_object.primitives.push_back(primitive);
collision_object.primitive_poses.push_back(box_pose);
collision_object.operation = collision_object.ADD;
```