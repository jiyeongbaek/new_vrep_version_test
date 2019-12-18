#include <iostream>
#include <memory>
#include <string>

#include "vrep_test/vrep_bridge.h"
#include "vrep_test/model/rbdl.h"

#include <fstream>
#include <std_msgs/Bool.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include "rosgraph_msgs/Clock.h"

using namespace std;

bool _kbhit()
{
    termios term;
    tcgetattr(0, &term);
    termios term2 = term;
    term2.c_lflag &= ~ICANON;
    tcsetattr(0, TCSANOW, &term2);
    int byteswaiting;
    ioctl(0, FIONREAD, &byteswaiting);
    tcsetattr(0, TCSANOW, &term);
    return byteswaiting > 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vrep_test");
    ros::NodeHandle nh;
    const double hz = 1000;

    vrep_bridge vb(nh, hz);

    // Start V-rep
    ros::Publisher start_pub = nh.advertise<std_msgs::Bool>("/startSimulation", 5);
    std_msgs::Bool start;
    start.data = true;
    while (!start_pub.getNumSubscribers() && ros::ok())
    {
    }
    start_pub.publish(start);
    sleep(1);
    vb.vrepEnableSyncMode();
    sleep(1);

    /* model Updater 생성 */
    std::shared_ptr<FrankaModelUpdater> panda_model = std::make_shared<FrankaModelUpdater>(vb.current_q_, vb.current_qdot_, vb.desired_torque_);
    //   std::map<std::string, std::shared_ptr<FrankaModelUpdater>> arms_data_;
    //   std::string left_arm_id = "panda_left";
    //   arms_data_.emplace(std::make_pair(left_arm_id, left_arm_data));
    bool isSimulationRun = false;
    bool exitflag = false;

    Matrix<double, 7, 1> torque_command;
    torque_command.setZero();

    ros::Time time = ros::Time::now();
    rosgraph_msgs::Clock time_data;
    ros::Publisher time_pub = nh.advertise<rosgraph_msgs::Clock>("/clock", 5);
    while (ros::ok() && !exitflag)
    {
        vb.read_vrep();
        // cout << "rbdl : " << panda_model->q_.transpose() << endl;
        // cout << "vrep : " << vb.current_q_.transpose() << endl;
        if (_kbhit())
        {
            int key = getchar();
            switch (key)
            {
            case '\t':
                if (isSimulationRun){
                    cout << "Simulation Pause " << endl;
                    isSimulationRun = false;}
                else{
                    cout << "Simulation Run" << endl;
                    isSimulationRun = true;}
                break;
            case 'q':
                isSimulationRun = false;
                exitflag = true;
                break;
            case 'i':
                break;
            }
        }
        if (isSimulationRun)
        {
            cout << "vrep : " << vb.current_q_.transpose() << endl;
            cout << "rbdl q : " << panda_model->q_.transpose() << endl;
            panda_model->updatemodel();
            panda_model->setTorque(torque_command);
            cout << "rbdl torque : " << panda_model->desired_torque_.transpose () << endl;
            cout << "mass matrix : \n" << panda_model->mass_matrix_ << endl;
            vb.write_robot();
            
            time = time + ros::Duration(0.001);
            time_data.clock = time;
            time_pub.publish(time_data);
            vb.wait();
        }
    }

    return 0;
}