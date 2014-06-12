#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction > TrajClient;

class RobotArm
{
private:
  // Action client for the joint trajectory action 
  // used to trigger the arm movement action
  TrajClient* traj_client_;

public:
  //! Initialize the action client and wait for action server to come up
  RobotArm() 
  {
    // tell the action client that we want to spin a thread by default
    traj_client_ = new TrajClient("/mitsubishi_arm/mitsubishi_arm_trajectory_controller/follow_joint_trajectory", true);

    // wait for action server to come up
    while(!traj_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the joint_trajectory_action server");
    }
  }

  //! Clean up the action client
  ~RobotArm()
  {
    delete traj_client_;
  }

  //! Sends the command to start a given trajectory
  void startTrajectory(control_msgs::FollowJointTrajectoryGoal goal)
  {
    // When to start the trajectory: 1s from now

    //goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    traj_client_->sendGoal(goal);
  }

  //! Generates a simple trajectory with two waypoints, used as an example
  /*! Note that this trajectory contains two waypoints, joined together
      as a single trajectory. Alternatively, each of these waypoints could
      be in its own trajectory - a trajectory can have one or more waypoints
      depending on the desired application.
  */
  control_msgs::FollowJointTrajectoryGoal armExtensionTrajectory()
  {
    ROS_INFO("computing trajectory");
    //our goal variable
    control_msgs::FollowJointTrajectoryGoal goal;

    // First, the joint names, which apply to all waypoints
    goal.trajectory.joint_names.push_back("j1");
    goal.trajectory.joint_names.push_back("j2");
    goal.trajectory.joint_names.push_back("j3");
    goal.trajectory.joint_names.push_back("j4");
    goal.trajectory.joint_names.push_back("j5");
    goal.trajectory.joint_names.push_back("j6");

    double joint_max_limit=1.0;
    int angle_resolution=10;
    double time_resolution=2.0;
    for(int i=0; i<angle_resolution;++i)
    {
       trajectory_msgs::JointTrajectoryPoint point;

	    // First trajectory point
	    // Positions
	    int ind = 0;
	    point.positions.resize(6);
	    point.positions[0] = 0.0;
        point.positions[1] = i*(joint_max_limit/angle_resolution);
	    point.positions[2] = 0.0;
	    point.positions[3] = 0.0;
	    point.positions[4] = 0.0;
	    point.positions[5] = 0.0;


	    // Velocities
	    point.velocities.resize(6);
	    for (size_t j = 0; j < 6; ++j)
	    {
	      point.velocities[j] = 0.0;
	    }
	    // To be reached 1 second after starting along the trajectory
	    point.time_from_start = ros::Duration(time_resolution*i);

	    // To be reached 2 seconds after starting along the trajectory

	    goal.trajectory.points.push_back(point);
    }
    ROS_INFO("DONE");
    //we are done; return the goal
    return goal;
  }

  //! Returns the current state of the action
  actionlib::SimpleClientGoalState getState()
  {

    std::cout << "RESULT:" << traj_client_->getResult()->error_code << std::endl;

    std::cout << "STATE:" << traj_client_->getState().toString() << std::endl;
    return traj_client_->getState();
  }
 
};

int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "robot_driver");

  RobotArm arm;
  // Start the trajectory
  arm.startTrajectory(arm.armExtensionTrajectory());
  // Wait for trajectory completion
  while(!arm.getState().isDone() && ros::ok())
  {
      std::cout << "RESULT:" << traj_client_->getResult()->error_code << std::endl;

    usleep(50000);
  }
}
