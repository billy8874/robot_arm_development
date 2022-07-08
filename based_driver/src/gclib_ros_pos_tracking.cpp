#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "based_driver/PosCmd.h"
#include "../include/gclib_ros/x_examples.h"
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>

class gc_obj{
  public:
    GCon g = 0;
    bool Connect(string ip);
    bool PosCmd(based_driver::PosCmd::Request  &req,
  	      based_driver::PosCmd::Response &res);
};

bool gc_obj::Connect(string ip){
  char buf[G_SMALL_BUFFER];
  string tmp = ip+" --subscribe ALL";
  cout << "Connecting to hardware.\n";
  x_e(GOpen(tmp.c_str(), &g));
  x_e(GInfo(g, buf, sizeof(buf))); //grab connection string
  cout << buf << '\n';
  return true;
}

//callback service
bool gc_obj::PosCmd(based_driver::PosCmd::Request &req, based_driver::PosCmd::Response &res){
  string tmp;
  
  trajectory_msgs::JointTrajectory::_points_type::iterator iter;
  char buf[1024]; //traffic buffer
	char* trimmed; //trimmed string pointer
 
  x_e(GCmd(g,"PT 1,1"));// start position tracking mode
  //x_e(GCmd(g,"SP 30000,30000"));
  x_e(GCmd(g,"AC 1073740800,1073740800"));
  x_e(GCmd(g,"DC 1073740800,1073740800"));
  
  for(iter=req.traj.points.begin(); iter!=req.traj.points.end(); iter++){
    //ROS_INFO("request: PA A = [%ld], PA B = [%ld]",(long int)(iter->positions[0]*3200/3.14159), (long int)(iter->positions[1]*3200/3.14159));
    if(iter->velocities[0]!=0){
      tmp = "SPA=" + to_string((long int)(iter->velocities[0]*3200/3.14159));
      x_e(GCmd(g,tmp.c_str()));// set up speed
    }
    if(iter->velocities[1]!=0){
      tmp = "SPB=" + to_string((long int)(iter->velocities[1]*3200/3.14159));
      x_e(GCmd(g,tmp.c_str()));// set up speed
    }
    
    tmp = "PA " + to_string((long int)(iter->positions[0]*3200/3.14159)) + "," + to_string((long int)(iter->positions[1]*3200/3.14159));
    x_e(GCmd(g,tmp.c_str()));// set up absolute position
    
    if(iter==req.traj.points.begin()){
      ((iter+1)->time_from_start - iter->time_from_start).sleep();// set up sleep time
    }
    else
      (iter->time_from_start - (iter-1)->time_from_start).sleep();// set up sleep time
      
    //cout << (double)iter->time_from_start.toNSec()/1000000000;
    //cout << ros::Time::now() << '\t' << (long int)(iter->positions[0]*3200/3.14159) << '\t' << (long int)(iter->positions[1]*3200/3.14159) << endl;
  }
  //cout << endl;
  //x_e(GCmd(g,"PT 0,0"));// end of position tracking mode
  
  res.res = true;
  //ROS_INFO("sending back response: [%d]", res.res);
  return true;
}	  

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gclib_ros");
  ros::NodeHandle n;
  
  // create gc object connecting to galil MT-100
  gc_obj g;
  g.Connect("192.168.1.11");
  cout<<"Connection Successful!"<<endl;
  
  ros::ServiceServer service = n.advertiseService("PosCmd_server", &gc_obj::PosCmd, &g);
  ROS_INFO("Ready to command motor.");
  ros::spin();

  return 0;
}