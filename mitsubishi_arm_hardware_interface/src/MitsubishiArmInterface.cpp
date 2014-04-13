#include <mitsubishi_arm_hardware_interface/MitsubishiArmInterface.h>
#include <sstream>
MitsubishiArmInterface::MitsubishiArmInterface() : n_priv("~")
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


    // Open File Descriptor
    USB = open( "/dev/ttyUSB0",  O_RDWR | O_NOCTTY);

    // Error Handling
    if ( USB < 0 )
    {
        std::cout << "Error " << errno << " opening " << "/dev/ttyUSB0" << ": " << strerror (errno) << std::endl;
    }

    // Configure Port
    memset (&tty, 0, sizeof tty);

    // Error Handling
    if ( tcgetattr ( USB, &tty ) != 0 )
    {
        std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
    }


    // Set Baud Rate
    cfsetospeed (&tty, B9600);
    cfsetispeed (&tty, B9600);

    long BAUD    =B9600;

    tty.c_cflag = BAUD | CRTSCTS | CS8 | CLOCAL | CREAD | PARENB;

    tty.c_cc[VMIN]=0;
    tty.c_cc[VTIME]=1;

    // Flush Port, then applies attributes
    tcflush( USB, TCIFLUSH );

    if ( tcsetattr ( USB, TCSANOW, &tty ) != 0)
    {
        std::cout << "Error " << errno << " from tcsetattr" << std::endl;
    }
    else
        std::cout << "connected successfuly" <<std::endl;
    usleep(100000);

}

MitsubishiArmInterface::~MitsubishiArmInterface()
{
    close(USB);
    std::cout << "close interface" << std::endl;
}

void MitsubishiArmInterface::readHW()
{
    // WRITE cmd to the robot
    unsigned char cmd[] = "1\r\n";
    int n_written = 0;

    do
    {
        n_written += write( USB, &cmd[n_written], 1 );
    }
    while (cmd[n_written-1] != '\n' && n_written > 0);

    char buf [256];
    memset (&buf, '\0', sizeof buf);
    //READ

    int n = 0;
    std::string response;

    do
    {
        n += read( USB, &buf, 1);
        response.append( buf );
    }
    while( buf[0] != '\n' && n > 0);


    response.clear();

    memset (&buf, '\0', sizeof buf);

    do
    {
        n += read( USB, &buf, 1);
        response.append( buf );
        //std::cout << "buff:"<<buf << std::endl;
    }
    while( buf[0] != '\n' && n > 0);

    std::stringstream convertor(response);

    char dummy_char;
    convertor >> dummy_char
              >> pos[0]
              >> dummy_char
              >> pos[1]
              >> dummy_char
              >> pos[2]
              >> dummy_char
              >> pos[3]
              >> dummy_char
              >> pos[4]
              >> dummy_char
              >> pos[5]
              >> dummy_char
              >> pos[6]
              >> dummy_char;

    // convert to radians and add to state
    for(int i=0; i< pos.size(); ++i)
    {
        pos[i]=DEG_TO_RAD*pos[i];
    }

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

    std::cout << "pos:"<< jnt_state_interface.getHandle("j6").getPosition() << std::endl;
}


void MitsubishiArmInterface::writeHW()
{
    // WRITE cmd to the robot
    unsigned char cmd[] = "2\r\n";
    int n_written = 0;

    do
    {
        n_written += write( USB, &cmd[n_written], 1 );
    }
    while (cmd[n_written-1] != '\n' && n_written > 0);

    char buf [256];
    memset (&buf, '\0', sizeof buf);
    //READ A

    int n = 0;
    std::string response;

    do
    {
        n += read( USB, &buf, 1);
        response.append( buf );
    }
    while( buf[0] != '\n' && n > 0);


    response.clear();

    memset (&buf, '\0', sizeof buf);

    // Write
    do
    {
        n += read( USB, &buf, 1);
        response.append( buf );
        //std::cout << "buff:"<<buf << std::endl;
    }
    while( buf[0] != '\n' && n > 0);

    std::stringstream convertor(response);

    char dummy_char;
    convertor >> dummy_char
              >> pos[0]
              >> dummy_char
              >> pos[1]
              >> dummy_char
              >> pos[2]
              >> dummy_char
              >> pos[3]
              >> dummy_char
              >> pos[4]
              >> dummy_char
              >> pos[5]
              >> dummy_char
              >> pos[6]
              >> dummy_char;

    // convert to radians and add to state
    for(int i=0; i< pos.size(); ++i)
    {
        pos[i]=DEG_TO_RAD*pos[i];
    }

    if(convertor.fail() == true)
    {
        // if the data string is not well-formatted do what ever you want here
    }


    if (n < 0)
    {
        std::cout << "Error reading: " << strerror(errno) << std::endl;
    }
    else if (n == 0)
    {
        std::cout << "Read nothing!" << std::endl;
    }
    else
    {
//        std::cout << "Response: " << response;
    }



}

/*
int main(int argc, char** argv)
{
    ros::init(argc, argv, std::string("mitsubishi_arm_interface"));

    MitsubishiArmInterface robot;
    controller_manager::ControllerManager cm(&robot);

    cm.loadController("joint_state_controller");
    std::vector<std::string> start_vec;
    start_vec.push_back("joint_state_controller");
    std::vector<std::string> stop_vec;

//    ros::Duration period;
//    ros::Time now=ros::Time::now();

//    period=now-now;

//    cm.update(now, period);


    ros::Time previous;
    ros::Rate r(10);
    while (ros::ok())
    {
        ros::Duration period;

        ros::Time now=ros::Time::now();
        period=now-previous;
        previous=now;
        robot.readHW();
        cm.update(now, period,true);
        cm.switchController(start_vec,stop_vec,2);

        //ROS_INFO_STREAM("period:"<< period);
        robot.writeHW();

        ros::spinOnce();

        r.sleep();
    }

    return 1;
}*/

