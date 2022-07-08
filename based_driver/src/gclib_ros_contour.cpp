#include "ros/ros.h"
#include "based_driver/PosCmd.h"
#include "../include/gclib_ros/x_examples.h"
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <cmath>

// store current position (joints)
long int cur_pos1 = 0;
long int cur_pos2 = 0;
long int cur_pos3 = 0;

// class to connect and command galil control card
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

// callback service
bool gc_obj::PosCmd(based_driver::PosCmd::Request &req, based_driver::PosCmd::Response &res){
  string tmp;
  int buff,dt;
  char buf[1024];
  char* trimmed;
  trajectory_msgs::JointTrajectory::_points_type::iterator iter;
  
  x_e(GCmd(g, "CM ABC"));//start contour mode
  x_e(GCmd(g, "DT 4"));//set a default time interval
  
  for(iter=req.traj.points.begin(); iter!=req.traj.points.end(); iter++){
    //check whether the buffer is empty
    x_e(GCmdT(g, "CM ?", buf, sizeof(buf), &trimmed));
    buff = atoi(trimmed);
    if(buff ==2){
      x_e(GCmd(g, "CD 0,0,0=0"));// end of contour mode temporarily
      while(buff != 32){
        x_e(GCmdT(g, "CM ?", buf, sizeof(buf), &trimmed));
        buff = atoi(trimmed);
      }
      x_e(GCmd(g, "CM ABC"));// re-start contour mode
      x_e(GCmd(g, "DT 4"));//re-set a default time interval
    }
    //set the first time interval individually
    if(iter==req.traj.points.begin())
      dt = round(log2(((iter+1)->time_from_start - iter->time_from_start).toNSec() / 1000000));
    else
      dt = round(log2((iter->time_from_start - (iter-1)->time_from_start).toNSec() / 1000000));
    //set time step to power of 2
    if(dt<1)
      dt = 1;
    else if(dt>8)
      dt = 8;
    
    //CD: write request into the buffer
    tmp = "CD " + to_string((long int)(iter->positions[0]*3200/3.14159)-cur_pos1) + "," + to_string((long int)(iter->positions[1]*3200/3.14159)-cur_pos2) 
      + "," + to_string((long int)(iter->positions[2]*3200/3.14159)-cur_pos3) + "=" + to_string(dt);
    x_e(GCmd(g,tmp.c_str()));
    //if(iter==req.traj.points.begin()) 
      //cout << ros::Time::now() << endl;
    cur_pos1 = (long int)(iter->positions[0]*3200/3.14159);//update current positions
    cur_pos2 = (long int)(iter->positions[1]*3200/3.14159);
    cur_pos3 = (long int)(iter->positions[2]*3200/3.14159);
    //cout << pow(2,dt) << '\t' << cur_pos1 << '\t' << cur_pos2 << '\t' << cur_pos3 << endl;
  }
  
  x_e(GCmd(g, "CD 0,0,0=0"));// end of contour mode
  
  res.res = true;
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
