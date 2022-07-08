#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "avoid_obstacle");
  ros::NodeHandle node_handle;

  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  // BEGIN_TUTORIAL
  
  // Setup
  // ^^^^^
  //
  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
  // are used interchangeably.
  static const std::string PLANNING_GROUP = "arm";

  // The :planning_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

  // We will use the :planning_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group =
      move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
      
  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script.
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to add a collision object in RViz");
  
  // Now let's define a collision object ROS message for the robot to avoid.
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_interface.getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object.id = "box1";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.1;
  primitive.dimensions[primitive.BOX_Y] = 0.1;
  primitive.dimensions[primitive.BOX_Z] = 0.2;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.4;
  box_pose.position.y = 0.1;
  box_pose.position.z = 0.0;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  // Now, let's add the collision object into the world
  // (using a vector that could contain additional objects)
  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  // Show text in RViz of status and wait for MoveGroup to receive and process the collision object message
  visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the cuboid appears in RViz");
  
  // delete the object from the world
  collision_objects.pop_back();
  std::vector<std::string> object_ids;
  object_ids.push_back(collision_object.id);
  //object_ids.push_back(object_to_attach.id);
  planning_scene_interface.removeCollisionObjects(object_ids);
  object_ids.pop_back();
  
  
  //Add Another Object to the world
  //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^  
  //
  //Now let's define a collision object ROS message for the robot to avoid.
  moveit_msgs::CollisionObject collision_object2;
  collision_object2.header.frame_id = move_group_interface.getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object2.id = "cylinder1";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive_cyl;
  primitive_cyl.type = primitive_cyl.CYLINDER;
  primitive_cyl.dimensions.resize(2);
  primitive_cyl.dimensions[primitive.CYLINDER_HEIGHT] = 0.2;
  primitive_cyl.dimensions[primitive.CYLINDER_RADIUS] = 0.05;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose cyl_pose;
  cyl_pose.orientation.w = 1.0;
  cyl_pose.position.x = 0.35;
  cyl_pose.position.y = -0.2;
  cyl_pose.position.z = 0.0;

  collision_object2.primitives.push_back(primitive_cyl);
  collision_object2.primitive_poses.push_back(cyl_pose);
  collision_object2.operation = collision_object2.ADD;

  //std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object2);

  // Now, let's add the collision object into the world
  // (using a vector that could contain additional objects)
  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);
  
  // Show text in RViz of status and wait for MoveGroup to receive and process the collision object message
  visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the cylinder appears in RViz");

  // delete the object from the world
  collision_objects.pop_back();
  //std::vector<std::string> object_ids;
  object_ids.push_back(collision_object2.id);
  //object_ids.push_back(object_to_attach.id);
  planning_scene_interface.removeCollisionObjects(object_ids);
  object_ids.pop_back();
  
  
  // Add Multiple Objects
  //^^^^^^^^^^^^^^^^^^^^^^
  //
  collision_objects.push_back(collision_object);
  collision_objects.push_back(collision_object2);

  // Now, let's add the collision object into the world
  // (using a vector that could contain additional objects)
  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  // Show text in RViz of status and wait for MoveGroup to receive and process the collision object message
  visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once both cyinder and cuboid appear in RViz");
  
  // delete the objects from the world
  collision_objects.pop_back();
  collision_objects.pop_back();
  object_ids.push_back(collision_object.id);
  object_ids.push_back(collision_object2.id);
  //object_ids.push_back(object_to_attach.id);
  planning_scene_interface.removeCollisionObjects(object_ids);
  object_ids.pop_back();
  object_ids.pop_back();
  
  visual_tools.publishText(text_pose, "delete object", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to remove all collision objects in RViz");

  // END
  //^^^^^

  ros::shutdown();
  return 0;
}
