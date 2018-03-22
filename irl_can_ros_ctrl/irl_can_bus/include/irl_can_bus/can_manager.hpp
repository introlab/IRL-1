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
#include "throttling_def.hpp"

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

        // Message queues: 
        //  - 1 msg_recv_queue (overall), with its own mutex.
        //  - 1 can_send_queue per interface.
        //  - 1 dev_send_queue per device (for throttling).
        // Both send queue vectors share the same mutex.
        using CANFramePtr = std::shared_ptr<CANFrame>;
#ifdef CAN_USE_SPINNING_MUTEX
        using QueueMutex  = SpinningMutex; 
#else
        using QueueMutex  = std::mutex;
#endif
        using QueueLock   = std::lock_guard<QueueMutex>;
        using Queue       = std::queue<CANFramePtr>; 

        static const int                     MAX_MSG_QUEUE_SIZE = 4096;
        Queue                                msg_recv_queue_;
        QueueMutex                           msg_recv_queue_mtx_;
        std::vector<Queue>                   can_send_queues_;
        QueueMutex                           send_queues_mtx_;
        std::array<Queue, MAX_CAN_DEV_COUNT> dev_send_queues_;
        std::array<int, MAX_CAN_DEV_COUNT>   device_queue_map_;

        // Mechanism for waiting on the reception queue.
        MutexType               wait_msgs_mtx_;
        std::condition_variable wait_msgs_cond_;

        // Throttling mechanism.
        using SchedTimeBase  = std::chrono::microseconds;
        using SchedClock     = std::chrono::high_resolution_clock;
        using SchedTimePoint = std::chrono::time_point<SchedClock>;

        std::vector<int>        throttled_devs_;   // List of throttled
                                                   // devices.
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
        /// \brief Period length (in ticks) for this device.
        std::array<int, MAX_CAN_DEV_COUNT> dev_period_length_;
        /// \brief Current tick in period for this device.
        ///
        /// Tick should be between 0 and (dev_period_length - 1).
        std::array<int, MAX_CAN_DEV_COUNT> dev_period_ticks_;


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
        /// \param td     The throttling definition for the device.
        void throttling(int dev_id, const ThrottlingDef& td);

        /// \brief Change the base period of the throttling scheduler.
        ///
        /// The default period at initialization is 100 us.
        ///
        /// \param period The period, in microseconds.
        void throttlingPeriod(std::chrono::microseconds period);

        /// \brief Return the current throttling base period.
        const std::chrono::microseconds& throttlingPeriod() const
        {
            return sched_period_;
        }

        /// \brief Pop a single CAN message from the reception queue.
        ///
        /// Thread safe.
        ///
        /// \return false if no messages where available.
        bool popOneMessage(LaboriusMessage& msg);

        /// \brief Push a single CAN message on the device send queue.
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

        void writeMem(unsigned int device_id,
                        unsigned int offset,
                        unsigned char* data,
                        unsigned int size,
                        unsigned int priority = 0);

    private:
        /// \brief Disabled copy constructor.
        CANManager(const CANManager&);

        bool shouldRun();
        void pushInternalEvent(const char v = 1);
        /// \brief Adds a single frame to the proper CAN send queue.
        void pushOnCANSendQueue(const CANFramePtr& f);
        void processFrame(const CANFrame& frame);
        void mainLoop();

        bool shouldThrottle(int dev_id) const;
        void schedLoop();

        /// \brief Push a single CAN frame on the interface send queue.
        void pushOneFrame(const CANFrame* frame);

    };
}

#endif

