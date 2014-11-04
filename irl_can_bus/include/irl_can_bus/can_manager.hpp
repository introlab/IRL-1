#ifndef CAN_MANAGER_HPP
#define CAN_MANAGER_HPP

#include <irl_can_bus/laborius_message.hpp>

#include <mutex>
#include <thread>
#include <condition_variable>
#include <vector>
#include <string>
#include <queue>

#include "can_tools.hpp"
#include "spinning_mutex.hpp"

namespace irl_can_bus
{
    class CANManager
    {
    private:
        using MutexType = std::mutex;

        std::vector<int> fds_;

        MutexType   mutex_;
        std::thread thread_;
        bool        running_;

        int         pipe_[2];

        // Message queues.
        using QueueLock = std::lock_guard<SpinningMutex>;
        static const int                   MAX_MSG_QUEUE_SIZE = 4096;
        std::queue<CANFrame>               msg_recv_queue_;
        SpinningMutex                      msg_recv_queue_mtx_;
        std::vector<std::queue<CANFrame>>  msg_send_queues_;
        SpinningMutex                      msg_send_queues_mtx_;
        std::array<int, 256>               device_queue_map_;

        // Mechanism for waiting on the reception queue.
        MutexType               wait_msgs_mtx_;
        std::condition_variable wait_msgs_cond_;

    public:
        /// \brief Constructor.
        ///
        /// \param
        CANManager(const std::vector<std::string> if_names); 

        ~CANManager();

        void stop();

        /// \brief Pop a single CAN message from the reception queue.
        ///
        /// Thread safe.
        ///
        /// \return false if no messages where available.
        bool popOneMessage(LaboriusMessage& msg);

        /// \brief Push a single CAN message on the send queue.
        ///
        /// Thread safe.
        void pushOneMessage(const LaboriusMessage& msg);

        /// \brief Blocking wait for available messages.
        ///
        /// The method returns instantly if messages are already available.
        /// Note that there are no guarantees on message availability after
        /// return from this call, as the condition variable when locked is
        /// released if termination of the CAN thread has been requested.
        void waitForMessages();

        /// \brief Push one memory request message on the queue.
        ///
        /// See requestMem in can_tools.hpp for details.
        void requestMem(unsigned int device_id,
                        unsigned int offset,
                        unsigned int size,
                        unsigned int priority = 0);

    private:
        /// \brief Disabled copy constructor.
        CANManager(const CANManager&);

        bool shouldRun();
        void pushInternalEvent(const char v = 1);
        void pushOnQueue(const CANFrame& f, int q);
        void processFrame(const CANFrame& frame);
        void loop();


    };
}

#endif

