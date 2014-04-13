#include <controller_manager/controller_manager.h>
#include <mitsubishi_arm_hardware_interface/MitsubishiArmInterface.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "testRobot");
    ros::NodeHandle node;

    MitsubishiArmInterface robot;
    controller_manager::ControllerManager cm(&robot, node);

    // read robot description from parameter server
    std::string robot_description_string;
    TiXmlDocument robot_description_xml;
    if (node.getParam("robot_description", robot_description_string))
        robot_description_xml.Parse(robot_description_string.c_str());
    else
    {
        ROS_ERROR("Could not load the robot description from the parameter server");
        return -1;
    }
    TiXmlElement *robot_description_root = robot_description_xml.FirstChildElement("robot");
    if (!robot_description_root)
    {
        ROS_ERROR("Could not parse the robot description");
        return -1;
    }

    // Initialize controller manager from robot description
    /*if (!cm.initXml(robot_description_root)){
    ROS_ERROR("Could not initialize controller manager");
    return -1;
  }*/

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::Time previous;

    ros::Rate rate(1000.0);
    while (ros::ok())
    {
        ros::Duration period;

        ros::Time now=ros::Time::now();
        period=now-previous;
        cm.update(now, period);

        rate.sleep();
    }

    spinner.stop();

    return 0;
}


