#include <stdlib.h>
#include <string>

#include <boost/algorithm/string.hpp>
#include <fmt/core.h>
#include <fmt/format.h>

#include <ros/ros.h>
#include <serial/serial.h>
#include <geometry_msgs/Point.h>
#include <std_srvs/Empty.h>

#include <rvc_ros/MoveCommand.h>
#include <rvc_ros/SetCommand.h>

using std::string;


using namespace rvc_ros;

serial::Serial *ser;
ros::Publisher pub;

bool move(MoveCommand::Request &req, MoveCommand::Response &resp) {
  ser->write(fmt::format("m {} {};", req.dest.x, req.dest.y));
  resp.moving = true;

  return true;
}


bool move_to(MoveCommand::Request &req, MoveCommand::Response &resp) {
  ser->write(fmt::format("M {} {};", req.dest.x, req.dest.y));
  resp.moving = true;

  return true;
}


bool halt(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) {
  ser->write("S");

  return true;
}


bool set_endeff(SetCommand::Request &req, SetCommand::Response &resp) {
  if (req.active) {
    ser->write("P");
  } else {
    ser->write("R");
  }

  return true;
}


bool set_led(SetCommand::Request &req, SetCommand::Response &resp) {
  if (req.active) {
    ser->write("L");
  } else {
    ser->write("O");
  }
  
  return true;
}


bool tap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) {
  ser->write("T");

  return true;
}


void publishPosition(const ros::TimerEvent &e) {
  if(ser->available() > 0) {
    geometry_msgs::Point msg;
    string str = ser->readline(128, "\n");
    std::vector<string> results;
    boost::split(results, str, [](char c){return c == '\t';});


    msg.x = std::stod(results[0]);
    msg.y = std::stod(results[1]);
    msg.z = 0;

    pub.publish(msg);
  }
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "rvc_ros");
  ros::NodeHandle nh;

  pub = nh.advertise<geometry_msgs::Point>("rvc_position", 1000);

  string port = "/dev/ttyUSB0";

  ser = new serial::Serial(port, 19200, serial::Timeout::simpleTimeout(1000));

  if (ser->isOpen())
    ROS_INFO("Opened serial port %s", port.c_string());
  else
    ROS_FATAL("Failed to open serial port %s", port.c_string());

  ros::Duration(5).sleep();

  ser->write("E");
  ser->write("H");

  while(ser->available() == 0)
    ros::Duration(0.5).sleep();

  ros::Timer position_timer = nh.createTimer(ros::Duration(0.2), &publishPosition);
  
  ros::ServiceServer move_serv = nh.advertiseService("move", &move);
  ros::ServiceServer move_to_serv = nh.advertiseService("move_to", &move_to);
  ros::ServiceServer halt_serv = nh.advertiseService("halt", &halt);
  ros::ServiceServer set_endeff_serv = nh.advertiseService("set_endeff", &set_endeff);
  ros::ServiceServer set_led_serv = nh.advertiseService("set_led", &set_led);
  ros::ServiceServer tap_serv = nh.advertiseService("tap", &tap);

  ros::spin();
  
  return 0;
}
