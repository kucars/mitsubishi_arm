#include <controller_manager/controller_manager.h>
#include <mitsubishi_arm_hardware_interface/MitsubishiArmInterface.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mitsubishi_arm_hardware_interface");
    ros::NodeHandle node;

    hardware_interface::JointStateInterface  jnt_state_interface_;
    hardware_interface::PositionJointInterface jnt_pos_interface_;

    std::string port;

    node.param<std::string>("mitsubishi_port", port, "/dev/ttyUSB1");

    MitsubishiArmInterface robot(port);
    robot.init(jnt_state_interface_, jnt_pos_interface_);
    controller_manager::ControllerManager cm(&robot, node);

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::Time previous=ros::Time::now();

    ros::Rate rate(1000.0);
    while (ros::ok())
    {
        ros::Duration period;
        robot.readHW();
        ros::Time now=ros::Time::now();
        period=now-previous;
        //std::cout << "period:"<<period<<std::endl;
        cm.update(now, period);
        robot.writeHW();
        rate.sleep();
    }

    spinner.stop();

    return 0;
}


