#include <mitsubishi_arm_hardware_interface/MitsubishiArmInterface.h>
#include <sstream>
MitsubishiArmInterface::MitsubishiArmInterface(std::string & port)
{
    //joint_state_pub=n_priv.advertise<sensor_msgs::JointState>( "joint_states", 1);

    pos.resize(joint_number);
    vel.resize(joint_number);
    eff.resize(joint_number);
    cmd.resize(joint_number);


    // connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle_j1("j1", &pos[0], &vel[0], &eff[0]);
    jnt_state_interface.registerHandle(state_handle_j1);

    hardware_interface::JointStateHandle state_handle_j2("j2", &pos[1], &vel[1], &eff[1]);
    jnt_state_interface.registerHandle(state_handle_j2);

    hardware_interface::JointStateHandle state_handle_j3("j3", &pos[2], &vel[2], &eff[2]);
    jnt_state_interface.registerHandle(state_handle_j3);

    hardware_interface::JointStateHandle state_handle_j4("j4", &pos[3], &vel[3], &eff[3]);
    jnt_state_interface.registerHandle(state_handle_j4);

    hardware_interface::JointStateHandle state_handle_j5("j5", &pos[4], &vel[4], &eff[4]);
    jnt_state_interface.registerHandle(state_handle_j5);

    hardware_interface::JointStateHandle state_handle_j6("j6", &pos[5], &vel[5], &eff[5]);
    jnt_state_interface.registerHandle(state_handle_j6);

    registerInterface(&jnt_state_interface);

    // connect and register the joint position interface
    hardware_interface::JointHandle pos_handle_j1(jnt_state_interface.getHandle("j1"), &cmd[0]);
    jnt_pos_interface.registerHandle(pos_handle_j1);

    hardware_interface::JointHandle pos_handle_j2(jnt_state_interface.getHandle("j2"), &cmd[1]);
    jnt_pos_interface.registerHandle(pos_handle_j2);

    hardware_interface::JointHandle pos_handle_j3(jnt_state_interface.getHandle("j3"), &cmd[2]);
    jnt_pos_interface.registerHandle(pos_handle_j3);

    hardware_interface::JointHandle pos_handle_j4(jnt_state_interface.getHandle("j4"), &cmd[3]);
    jnt_pos_interface.registerHandle(pos_handle_j4);

    hardware_interface::JointHandle pos_handle_j5(jnt_state_interface.getHandle("j5"), &cmd[4]);
    jnt_pos_interface.registerHandle(pos_handle_j5);

    hardware_interface::JointHandle pos_handle_j6(jnt_state_interface.getHandle("j6"), &cmd[5]);
    jnt_pos_interface.registerHandle(pos_handle_j6);

    registerInterface(&jnt_pos_interface);

    cmd=pos;

    cmd_previous=cmd;

}

MitsubishiArmInterface::~MitsubishiArmInterface()
{
    close(USB);
}

void MitsubishiArmInterface::readHW()
{
    ros::Duration(0.025).sleep();

    // convert to radians and add to state
/*    for(int i=0; i< pos.size(); ++i)
    {
        pos[i]=cmd[i];
    }*/


    eff[0]=0.0;
    eff[1]=0.0;
    eff[2]=0.0;
    eff[3]=0.0;
    eff[4]=0.0;
    eff[5]=0.0;

    vel[0]=0.0;
    vel[1]=0.0;
    vel[2]=0.0;
    vel[3]=0.0;
    vel[4]=0.0;
    vel[5]=0.0;
}


void MitsubishiArmInterface::writeHW()
{
    if(isEqual(cmd_previous[0],cmd[0],0.00001)&&
       isEqual(cmd_previous[1],cmd[1],0.00001)&&
       isEqual(cmd_previous[2],cmd[2],0.00001)&&
       isEqual(cmd_previous[3],cmd[3],0.00001)&&
       isEqual(cmd_previous[4],cmd[4],0.00001)&&
       isEqual(cmd_previous[5],cmd[5],0.00001))
    {
        //std::cout << "new command to write"<< cmd<< std::endl;
        cmd_previous=cmd;
        return;
    }
    cmd_previous=cmd;
    for(int i=0; i< pos.size(); ++i)
    {
        pos[i]=cmd[i];
    }
    static int new_command_count=0;
    new_command_count++;
    std::cout << "new command:"<< new_command_count << " " <<cmd[1] << std::endl;
    ros::Duration(0.025).sleep();
}


