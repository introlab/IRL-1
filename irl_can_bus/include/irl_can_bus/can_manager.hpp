#ifndef CAN_MANAGER_HPP
#define CAN_MANAGER_HPP

#include <irl_can_bus/laborius_message.hpp>

#include <mutex>
#include <thread>
#include <condition_variable>
#include <vector>
#include <string>
#include <queue>
#include <chrono>
#include <boost/circular_buffer.hpp>

#include "can_tools.hpp"
#include "spinning_mutex.hpp"

namespace irl_can_bus
{
    /// \brief A class that gives access to one or more CAN bus interfaces and
    /// manages input and output messages with optional throttling per device.
    ///
    class CANManager
    {
    private:
        using MutexType = std::mutex;

        std::vector<int> fds_;

        MutexType   mutex_;
        std::thread main_thread_;
        bool        running_;

        int         pipe_[2];

        // Message queues.
        using CANFramePtr = CANFrame*;
#ifdef CAN_USE_SPINNING_MUTEX
        using QueueMutex  = SpinningMutex; 
#else
        using QueueMutex  = std::mutex;
#endif
        using QueueLock   = std::lock_guard<QueueMutex>;
        using Queue       = std::queue<CANFramePtr>; 

        static const int                   MAX_MSG_QUEUE_SIZE = 4096;
        Queue                              msg_recv_queue_;
        QueueMutex                         msg_recv_queue_mtx_;
        std::vector<Queue>                 msg_send_queues_;
        QueueMutex                         msg_send_queues_mtx_;
        std::array<int, MAX_CAN_DEV_COUNT> device_queue_map_;

        // Mechanism for waiting on the reception queue.
        MutexType               wait_msgs_mtx_;
        std::condition_variable wait_msgs_cond_;

        // Throttling mechanism.
        using SchedTimeBase  = std::chrono::microseconds;
        using SchedClock     = std::chrono::high_resolution_clock;
        using SchedTimePoint = std::chrono::time_point<SchedClock>;

        std::vector<Queue>      throttled_queues_; // Stores throttled
                                                   // messages.
        SchedTimeBase           sched_period_;     // Base scheduler period.
        MutexType               sched_mtx_;        // For condition variable.
        std::condition_variable sched_cond_;       // Used to yield when not 
                                                   // throttling.
        std::thread             sched_thread_;

        /// \brief How many messages can be sent per base time period.
        ///
        /// Zero (or lower) disables throttling.
        std::array<int, MAX_CAN_DEV_COUNT> max_sent_per_period_;
        /// \brief How many messages have been sent in the current period.
        std::array<int, MAX_CAN_DEV_COUNT> cur_sent_per_period_;

    public:
        /// \brief Constructor.
        ///
        /// \param
        CANManager(const std::vector<std::string> if_names); 

        ~CANManager();

        void stop();

        /// \brief Adjust throttling for a single CAN device.
        ///
        /// \param dev_id The device ID.
        /// \param count  The amount of messages that can be sent in a single
        ///               base time period (see throttlingPeriod).
        void throttling(int dev_id, int count);

        /// \brief Change the base period of the throttling scheduler.
        ///
        /// The default period at initialization is 100 us.
        ///
        /// \param period The period, in microseconds.
        void throttlingPeriod(std::chrono::microseconds period);

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
        ///
        /// \return true if messages are available, false if it timed out.
        bool waitForMessages();

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
        void mainLoop();

        bool shouldThrottle(int dev_id) const;
        void schedLoop();


    };
}

#endif

