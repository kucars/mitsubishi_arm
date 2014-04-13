#include <controller_manager/controller_manager.h>
#include <mitsubishi_arm_hardware_interface/MitsubishiArmInterface.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mitsubishi_arm_hardware_interface");
    ros::NodeHandle node;

    MitsubishiArmInterface robot;
    controller_manager::ControllerManager cm(&robot, node);

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::Time previous=ros::Time::now();

    ros::Rate rate(1000.0);
    while (ros::ok())
    {
        ros::Duration period;
        //robot.readHW();
        ros::Time now=ros::Time::now();
        period=now-previous;
        cm.update(now, period);
        robot.writeHW();
        rate.sleep();
    }

    spinner.stop();

    return 0;
}


