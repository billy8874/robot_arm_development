# Based driver &middot; [![ROS noetic](https://img.shields.io/badge/ROS-noetic-blue)](http://wiki.ros.org/noetic) [![Moveit 1 noetic](https://img.shields.io/badge/Moveit%201-noetic-blue)](https://ros-planning.github.io/moveit_tutorials/) [![Galil control card B140](https://img.shields.io/badge/Galil-B140-orange)](http://120.109.165.16/smartiot/IOT_Prj/Galil_DMC-B140_%E5%A6%8F%E8%9A%9A%E5%BF%92%EF%81%A4.pdf)

This is a ROS package that connects the Moveit controller with the Galil motion control card. Messages are passed between the moveit and Galil cards via the action server and client, and use Gclib as an API to issue commands to the control card.

## Launch
```
roslaunch based_driver moveit_to_galil.launch
```
This launch file provide two solutions to control motors via Galil control card, Position tracking mode and Contour mode.
```html
<launch>
  <!--node name="PosCmd_server" pkg="driver_action_server" type="gclib_ros_pos_tracking" output="screen"/-->
  <node name="PosCmd_server" pkg="based_driver" type="gclib_ros_contour" output="screen"/>
  <node name="fake_feedback" pkg="based_driver" type="fake_motor_feedback_pub" output="screen"/>
  <node name="action_for_moveit" pkg="based_driver" type="driver_action_server" output="screen"/>
  <include file="$(find my_moveit_controller)/launch/demo.launch" />
</launch>
```

## Code

### Gclib C++ command examples

#### Position tracking mode:
```cpp
Cmd(g, "PT 1");//start pos. tracking mode
Cmd(g, "PT 𝑥1");//assign an absolute position 
//motion start immediately after a PT command
Sleep(∆𝑡01);//wait motion to complete
Cmd(g, "PT 𝑥2");//new target position
Sleep(∆𝑡12);
Cmd(g, "PT 𝑥3");
Sleep(∆𝑡23);
Cmd(g, "PT 0");//end of pos. tracking mode
```

#### Contour mode:
```cpp
Cmd(g, "CM X");//start contour mode on X-axis
Cmd(g, "CD ∆𝑥01 = ∆𝑡01");
//write displacement and time step into the buffer
Cmd(g, "CD ∆𝑥12 = ∆𝑡12");
Cmd(g, "CD ∆𝑥23 = ∆𝑡23");
Cmd(g, "CD 0=0");//end of contour mode
//motion start after this command
```