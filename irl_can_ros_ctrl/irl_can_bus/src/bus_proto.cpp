#include <irl_can_bus/can_manager.hpp>
#include <iostream>
#include <signal.h>
#include <unistd.h>

namespace {
    std::unique_ptr<irl_can_bus::CANManager> manager_;
    bool                                     running_;

    void sigHandler(int sig)
    {
        std::cerr << "SIGINT caught." << std::endl;
        manager_->stop();
        running_ = false;
    }
}


int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cerr << "Usage: " 
                  << argv[0] 
                  << " if_name_0 [if_name_1] [...]"
                  << std::endl;
        exit(-1);
    }

    std::vector<std::string> if_names;
    if_names.reserve(argc - 1);
    for (int i = 1; i < argc; ++i) {
        if_names.push_back(argv[i]);
    }

    signal(SIGINT, sigHandler);

    manager_.reset(new irl_can_bus::CANManager(if_names));

    running_ = true;
    while (running_) {
        irl_can_bus::LaboriusMessage msg;
        while (manager_->popOneMessage(msg)) {
            std::cerr << "Got one message from " << msg.msg_dest << std::endl;
        }
        sleep(100);
    }

}

