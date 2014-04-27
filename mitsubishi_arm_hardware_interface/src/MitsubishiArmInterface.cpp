#include <mitsubishi_arm_hardware_interface/MitsubishiArmInterface.h>
#include <sstream>
MitsubishiArmInterface::MitsubishiArmInterface()
{
    //joint_state_pub=n_priv.advertise<sensor_msgs::JointState>( "joint_states", 1);

    pos.resize(joint_number);
    vel.resize(joint_number);
    eff.resize(joint_number);
    cmd.resize(joint_number);
    cmd_previous.resize(joint_number);


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
    cfsetospeed (&tty, B19200);
    cfsetispeed (&tty, B19200);

    long BAUD    =B19200;

    tty.c_cflag = BAUD | CRTSCTS | CS8 | CLOCAL | CREAD | PARENB;

    tty.c_cc[VMIN]=0;
    tty.c_cc[VTIME]=20;

    // Flush Port, then applies attributes
    tcflush( USB, TCIFLUSH );

    if ( tcsetattr ( USB, TCSANOW, &tty ) != 0)
    {
        std::cout << "Error " << errno << " from tcsetattr" << std::endl;
    }
    else
        std::cout << "connected successfuly" <<std::endl;
    usleep(10000);

    char buf [256];
    read( USB, &buf, 1); // CLEAN BUFFER
    readHW();

    cmd=pos;
    cmd_previous=cmd;
    // convert to radians and add to state
    for(int i=0; i< pos.size(); ++i)
    {
        std::cout << cmd[i] << std::endl;
    }


    std::cout << "Init done!" << '\n';


}

MitsubishiArmInterface::~MitsubishiArmInterface()
{
    close(USB);
}

void MitsubishiArmInterface::readHW()
{

    //boost::mutex::scoped_lock lock(io_mutex);
    //std::cout << "reading" << std::endl;

    // WRITE READ to robot
    unsigned char cmd_msg[] = "1\r\n";
    int n_written = 0;

    do
    {
        n_written += write( USB, &cmd_msg[n_written], 1 );
    }
    while (cmd_msg[n_written-1] != '\n');

    // READ RESPONSE (R)
    char buf [256];
    memset (&buf, '\0', sizeof buf);
    int n = 0;
    std::string response;

    do
    {
        n += read( USB, &buf, 1);
        response.append( buf );
    }
    while( buf[0] != '\n');

    if (response.find("R\r\n") == std::string::npos)
    {
        std::cout << "didn-t find R!" << '\n';
        exit(-1);
    }

    response.clear();
    // END READ

    // READ JOINTS STATE
    n_written = 0;
    memset (&buf, '\0', sizeof buf);

    do
    {
        n_written += read( USB, &buf, 1);
        response.append( buf );

    }
    while( buf[0] != '\n');

    std::stringstream convertor(response);

    // READ RESPONSE (E)
    memset (&buf, '\0', sizeof buf);
    n = 0;

    do
    {
        n += read( USB, &buf, 1);
        response.append( buf );
    }
    while( buf[0] != '\n');

    if (response.find("E\r\n") == std::string::npos)
    {
        std::cout << "didn-t find E!" << '\n';
        exit(-1);
    }
    // END READ


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
        pos[i]=pos[i]*(DEG_TO_RAD);
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
        //std::cout << "no new commands to write"<< std::endl;
        cmd_previous=cmd;
        return;
    }
    static int new_command_count=0;
    new_command_count++;
    std::cout << "new command:"<< new_command_count << std::endl;
    //boost::mutex::scoped_lock lock(io_mutex);
    // WRITE MOVE to robot
    unsigned char cmd_msg[] = "2\r\n";
    int n_written = 0;

    do
    {
        n_written += write( USB, &cmd_msg[n_written], 1 );
    }
    while (cmd_msg[n_written-1] != '\n');

    // READ RESPONSE (M)
    char buf [256];
    memset (&buf, '\0', sizeof buf);
    int n = 0;
    std::string response;

    do
    {
        n += read( USB, &buf, 1);
        response.append( buf );
    }
    while( buf[0] != '\n');

    if (response.find("M\r\n") == std::string::npos)
    {
        std::cout << "didn-t find M!" << '\n';
        exit(-1);
    }

    response.clear();
    // END READ

    std::stringstream write_msg;
    //std::cout << cmd[1] << std::endl;
    write_msg << double(cmd[0]) << "," <<cmd[1] << "," << cmd[2] << "," << cmd[3] << "," << cmd[4] << "," << cmd[5] << "\r\n";

    std::string write_str=write_msg.str();
    //std::cout << "writing command:" << write_str<< std::endl;

    // Write command
    write( USB, write_str.c_str(), write_str.size());
    cmd_previous=cmd;
    // READ RESPONSE (E)
    memset (&buf, '\0', sizeof buf);
    n = 0;
    //std::cout << "getting respoinse"<< std::endl;

    do
    {
        n += read( USB, &buf, 1);
        response.append( buf );
    }
    while( buf[0] != '\n');

    if (response.find("E\r\n") == std::string::npos)
    {
        std::cout << "didn-t find E!" << '\n';
        exit(-1);
    }


    // END READ
}


