#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "../include/gclib_ros/x_examples.h"

class gc_obj{
  public:
    GCon g = 0;
    int Jstate1, Jstate2, Jstate3;
    bool Connect(string ip);
    void getData();
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

void gc_obj::getData(){
  char buf[1024]; //traffic buffer
	char* trimmed; //trimmed string pointer
  // get current positions(fake)
  x_e(GCmdT(g, "TD A", buf, sizeof(buf), &trimmed));
  Jstate1 = atoi(trimmed);
  x_e(GCmdT(g, "TD B", buf, sizeof(buf), &trimmed));
  Jstate2 = atoi(trimmed);
  x_e(GCmdT(g, "TD C", buf, sizeof(buf), &trimmed));
  Jstate3 = atoi(trimmed);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fake_feedback");
  ros::NodeHandle n;
  sensor_msgs::JointState js;
  ros::Publisher pub_joint;
  
  int cur1,cur2,cur3;// to store current positions
  
  // create gc object connecting to galil B140
  gc_obj g;
  g.Connect("192.168.1.11");
  cout<<"Connection Successful!"<<endl;
  
  // joint state publish data initialize
  js.name.resize(3);
  js.position.resize(3);
  js.name[0] = "Joint1";
  js.name[1] = "Joint2";
  js.name[2] = "Joint3";
  
  pub_joint = n.advertise<sensor_msgs::JointState>("/move_group/myarm_controller_joint_states", 10);// advertise joint state publisher
  ros::Rate loop_rate(100);// setup rate
  
  g.getData();
  cur1 = g.Jstate1;// update current status
  cur2 = g.Jstate2;
  cur3 = g.Jstate3;
  
  while (ros::ok()){
    g.getData();
    if((cur1==g.Jstate1 && cur2==g.Jstate2) && cur3==g.Jstate3)
      continue;// pass if positions do not change
    //cout << ros::Time::now() << '\t' << g.Jstate1 << '\t' << g.Jstate2 << endl;
    js.position.clear();
    js.position.push_back((double)g.Jstate1/3200*3.14159);
    js.position.push_back((double)g.Jstate2/3200*3.14159);
    js.position.push_back((double)g.Jstate3/3200*3.14159);
    js.header.stamp = ros::Time::now();
    pub_joint.publish(js);// publish joint state
    ros::spinOnce();
    loop_rate.sleep();
    cur1 = g.Jstate1;// update joint state
    cur2 = g.Jstate2;
    cur3 = g.Jstate3;
  }

  return 0;
}