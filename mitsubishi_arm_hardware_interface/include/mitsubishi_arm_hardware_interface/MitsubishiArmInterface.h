#include <stdio.h>      // standard input / output functions
#include <stdlib.h>
#include <string.h>     // string function definitions
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions
#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <sensor_msgs/JointState.h>
#include <controller_manager/controller_manager.h>

#define PI 3.14159265359
#define DEG_TO_RAD PI/180.0

class MitsubishiArmInterface : public hardware_interface::RobotHW
{
public:
  MitsubishiArmInterface();
  ~MitsubishiArmInterface();
  void readHW();
  void writeHW();
  ros::NodeHandle n_priv;
  int USB;

private:
  static const unsigned int joint_number=6;
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  std::vector<double> cmd;
  std::vector<double> pos;
  std::vector<double> vel;
  std::vector<double> eff;


  // tty specific communication

  struct termios tty;

};

