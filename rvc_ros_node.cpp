// C/C++ Headers
#include <stdlib.h>
#include <string>

// Utility library headers
#include <boost/algorithm/string.hpp>
#include <fmt/core.h>
#include <fmt/format.h>

// ROS Headers
#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>


using std::string;


serial::Serial *ser;
ros::Publisher pub;


// Absolute movement subscriber callback
void move(const sensor_msgs::JointState &msg) {
  ser->write(fmt::format("m {} {};", msg.position[0], msg.position[1]));
}


// Relative movement subscriber callback
void move_to(const sensor_msgs::JointState &msg) {
  ser->write(fmt::format("M {} {};", msg.position[0], msg.position[1]));
}


// Halt service handler
bool halt(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) {
  ser->write("S");

  return true;
}


// End effector state set service handler
bool set_endeff(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp) {
  if (req.data) {
    ser->write("P");
    resp.message = "End Effector Pressed";
  } else {
    ser->write("R");
    resp.message = "End Effector Released";
  }

  resp.success = true;

  return true;
}


// LED state set service handler
bool set_led(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp) {
  if (req.data) {
    ser->write("L");
    resp.message = "LED Lit";
  } else {
    ser->write("O");
    resp.message = "LED Off";
  }

  resp.success = true;
  
  return true;
}


// End effector tap action service handler
bool tap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) {
  ser->write("T");

  return true;
}


// Position publisher timer event
void publishPosition(const ros::TimerEvent &e) {
  unsigned static int s = 0;

  // If serial has data available
  if(ser->available() > 0) {
    sensor_msgs::JointState msg;

    // Read a line, split by tab
    string str = ser->readline(128, "\n");
    std::vector<string> results;
    boost::split(results, str, [](char c){return c == '\t';});

    // Assign message data
    msg.header.seq = s;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "rvc";

    msg.name.resize(2);
    msg.name[0] = "x";
    msg.name[1] = "y";

    msg.position.resize(2);
    msg.position[0] = std::stod(results[0]);
    msg.position[1] = std::stod(results[1]);

    // Publish and increment sequence counter
    pub.publish(msg);
    s++;
  }
}


int main(int argc, char **argv) {
  // Initialize ROS
  ros::init(argc, argv, "rvc_ros");
  ros::NodeHandle nh;


  // Open serial port to device
  string port = "/dev/ttyUSB0";
  ser = new serial::Serial(port, 19200, serial::Timeout::simpleTimeout(1000));

  // Check that port is open, exit if not
  if (ser->isOpen())
    ROS_INFO("Opened serial port %s", port.c_str());
  else {
    ROS_FATAL("Failed to open serial port %s", port.c_str());
    ros::shutdown();
    return 0;
  }

  // Wait for device to ready
  ros::Duration(4).sleep();

  // Enable motors and perform auto-home
  ser->write("E");
  ser->write("H");

  // Wait for homing to complete
  while(ser->available() == 0)
    ros::Duration(0.5).sleep();


  // Define subscribers, publishers, and services
  ros::Timer position_timer = nh.createTimer(ros::Duration(0.2), &publishPosition);

  pub = nh.advertise<sensor_msgs::JointState>("setpoint_js", 1000);
  
  ros::Subscriber move_sub = nh.subscribe("move_jp", 1000, &move);
  ros::Subscriber move_to_sub = nh.subscribe("move_jr", 1000, &move_to);
  
  ros::ServiceServer halt_serv = nh.advertiseService("halt", &halt);
  ros::ServiceServer set_endeff_serv = nh.advertiseService("set_endeff", &set_endeff);
  ros::ServiceServer set_led_serv = nh.advertiseService("set_led", &set_led);
  ros::ServiceServer tap_serv = nh.advertiseService("tap", &tap);

  // Release flow control to ROS
  ros::spin();
  
  return 0;
}
