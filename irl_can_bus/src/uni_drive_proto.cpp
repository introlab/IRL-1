#include <irl_can_bus/can_robot.hpp>
#include <iostream>
#include <signal.h>
#include <unistd.h>
#include "can_base_macros.h"

namespace {
    using namespace irl_can_bus;

    void requestMem(LaboriusMessage& msg,
                    unsigned int     device_id,
                    unsigned int     offset,
                    unsigned int     size, 
                    unsigned char    priority = 0)
    {
        msg.msg_priority = priority;
        msg.msg_type = CAN_TYPE_REQUEST_DATA;
        msg.msg_cmd = offset;
        msg.msg_dest = device_id;
        msg.msg_boot = (CAN_REQUEST_RAM << 1) | (CAN_REQUEST_READ);

        msg.msg_remote = 1;
        msg.msg_data_length = size;
    }

    class UniDrive: public CANRobotDevice
    {
    private:
        int  pos_;
        bool pos_ready_;

        enum
        {
            MODE_VARIABLE_OFFSET            = 0,
            MODE_VARIABLE_NORMAL            = 1,
            MODE_VARIABLE_IDLE              = 0,
            DRIVE_STATE_OFFSET              = 8,
            SETPOINT_VARIABLE_OFFSET        = 10,
            MAX_SETPOINT_VARIABLE_OFFSET    = 14,
            MIN_SETPOINT_VARIABLE_OFFSET    = 18,
            SPEED_VARIABLE_OFFSET           = 66,
            POSITION_VARIABLE_OFFSET        = 74,
            TORQUE_VARIABLE_OFFSET          = 86,
            ADMITTANCE_M_OFFSET             = 158,
            ADMITTANCE_B_OFFSET             = 162,
            ADMITTANCE_K_OFFSET             = 166,
            TORQUE_OFFSET_OFFSET            = 210,
            POSITION_OFFSET_OFFSET          = 214,
            POSITION_TO_RAD_VARIABLE_OFFSET = 226,
            TORQUE_TO_NM_VARIABLE_OFFSET    = 230,
            TIMEBASE_VARIABLE_OFFSET        = 238
        };

    public:
        UniDrive(int dev_id): 
            CANRobotDevice(dev_id),
            pos_(0),
            pos_ready_(false)
        {
        }

        void enable(CANManager& can)
        {
            CANRobotDevice::enable(can);
        }

        void enableCtrl(CANManager& can)
        {
            CANRobotDevice::enableCtrl(can);

        }

        void disableCtrl(CANManager& can)
        {
            CANRobotDevice::disableCtrl(can);
        }

        void disable(CANManager& can)
        {
            CANRobotDevice::disable(can);
        }

        void requestState(CANManager& can)
        {
            LaboriusMessage msg_req;
            requestMem(msg_req, 
                       deviceID(),
                       POSITION_VARIABLE_OFFSET, 
                       sizeof(int));
            can.pushOneMessage(msg_req);
            pos_ready_ = false;
        }

        bool stateReady()
        {
            return pos_ready_;
        }

        void processMsg(const LaboriusMessage& msg)
        {
            switch(msg.msg_cmd) {
                case POSITION_VARIABLE_OFFSET:
                    pos_ = *((int*)msg.msg_data);
                    pos_ready_ = true;
                break;

                default:
                break;
            };
        }

        void update()
        {
            std::cout << "Pos: " << pos_ << std::endl;
        }
    };

    std::unique_ptr<irl_can_bus::CANRobot> robot_;
    bool                                   running_;

    void sigHandler(int sig)
    {
        std::cerr << "SIGINT caught." << std::endl;
        robot_->stop();
        running_ = false;
    }


}


int main(int argc, char** argv)
{
    if (argc < 3) {
        std::cerr << "Usage: " 
                  << argv[0] 
                  << " device_id if_name_0 [if_name_1] [...]"
                  << std::endl;
        exit(-1);
    }

    int device_id = atoi(argv[1]);
    std::vector<std::string> if_names;
    if_names.reserve(argc - 2);
    for (int i = 2; i < argc; ++i) {
        if_names.push_back(argv[i]);
    }

    signal(SIGINT, sigHandler);

    robot_.reset(new irl_can_bus::CANRobot(if_names));
    //manager_.reset(new irl_can_bus::CANManager(if_names));

    std::shared_ptr<UniDrive> drive(new UniDrive(device_id));

    robot_->registerCtrlCB(&UniDrive::update, drive.get());
    robot_->addDevice(drive);

    robot_->start();

    running_ = true;
    while (running_) {
        robot_->loopOnce();
        usleep(1e5);
    }

    robot_->stop();

}

