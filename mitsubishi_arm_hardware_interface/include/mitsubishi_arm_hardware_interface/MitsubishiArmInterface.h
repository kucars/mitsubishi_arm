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
#include <joint_trajectory_controller/hardware_interface_adapter.h>

#include <hardware_interface/robot_hw.h>
#include <sensor_msgs/JointState.h>
#include <controller_manager/controller_manager.h>
#include <boost/thread/mutex.hpp>
#include <math.h>
#define PI_ 3.14159265359
#define DEG_TO_RAD PI_/180.0

bool isEqual(double & a, double & b, double threshold)
{
    return fabs(a-b)<threshold;

}

class MitsubishiArmInterface : public hardware_interface::RobotHW
{
public:
    MitsubishiArmInterface();
    ~MitsubishiArmInterface();

    bool init();

    bool init(hardware_interface::JointStateInterface &jnt_state_interface_,
              hardware_interface::PositionJointInterface &jnt_pos_interface_);



    void readHW();
    void writeHW();
    int USB;

private:
    static const unsigned int joint_number=6;

    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::PositionJointInterface jnt_pos_interface;
    std::vector<double> cmd;
    std::vector<double> pos;
    std::vector<double> vel;
    std::vector<double> eff;
    std::vector<double> cmd_previous;
    //boost::mutex io_mutex;
    // tty specific communication

    struct termios tty;

};

