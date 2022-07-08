#include <ros/ros.h>
#include "sensor_msgs/JointState.h"
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include "based_driver/PosCmd.h"
#include "../include/gclib_ros/x_examples.h"

using namespace std;
ros::ServiceClient *clientPtr; // to store a client pointer

class JointTrajectoryActionServer{
  public:
    JointTrajectoryActionServer(std::string name):
      as_(nh_, name, false), action_name_(name){
    	// register callback for goal
    	as_.registerGoalCallback(boost::bind(&JointTrajectoryActionServer::goalCallback, this));
    	as_.start();
      // declare fake states to joint_state_publisher because we have no feedback from motor for now
      pub_joint = nh_.advertise<sensor_msgs::JointState>("/move_group/myarm_controller_joint_states", 10);
      js.name.resize(3);
      js.position.resize(3);
      js.name[0] = "Joint1";
      js.name[1] = "Joint2";
      js.name[2] = "Joint3";
      ROS_INFO("-------action start!-------");
    }
    ~JointTrajectoryActionServer(void){}

    // when a trajectory command comes, this function will be called.
    void goalCallback(){
	    // accept new goal form an action client
      control_msgs::FollowJointTrajectoryGoal::_trajectory_type trajectory;
      trajectory = as_.acceptNewGoal()->trajectory;
      trajectory_msgs::JointTrajectory::_points_type::iterator iter;// trajectory iterator
      
      // call motor controller service
      based_driver::PosCmd srv;
      srv.request.traj = trajectory;
      
      // publish fake joint states
      iter = trajectory.points.end() - 1;
      js.position.clear();
      js.position.push_back(iter->positions[0]);
      js.position.push_back(iter->positions[1]);
      js.position.push_back(iter->positions[2]);
      js.header.stamp = ros::Time::now();
      pub_joint.publish(js);
      
      ros::ServiceClient client = (ros::ServiceClient)*clientPtr; //dereference the clientPtr
  
      if(client.call(srv)){
        //ROS_INFO("Motor rotate successfully!");
        ROS_INFO("target: joint1 = %ld, joint2 = %ld", (long int)(iter->positions[0]*3200/3.14159), (long int)(iter->positions[0]*3200/3.14159));
      }   
      else{
        ROS_ERROR("Failed to call service gclib_ros");    
      }
      
      // return after executed successfully
      as_.setSucceeded(result_);
    }

  protected:
    // for fake joint state publish
    sensor_msgs::JointState js;
    ros::Publisher pub_joint;
    // for action server
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>::Result result_;
    std::string action_name_;
};

int main(int argc, char** argv){
	ros::init(argc,argv, "my_driver");
  ros::NodeHandle n;
  
  //create the client to call motor control service
  ros::ServiceClient client = n.serviceClient<based_driver::PosCmd>("PosCmd_server");
  clientPtr = &client; //give the address of the client to the clientPtr
  
	JointTrajectoryActionServer srv("arm_controller/follow_joint_trajectory");
	ros::spin();
  
	return 0;
}