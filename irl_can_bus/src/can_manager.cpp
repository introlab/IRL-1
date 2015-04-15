#include <irl_can_bus/can_manager.hpp>

#include <iostream>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <unistd.h>
#include <cassert>
#include <cerrno>

#include <linux/can/error.h>

using namespace irl_can_bus;


CANManager::CANManager(const std::vector<std::string> if_names): 
    running_(false),
    sched_period_(100)
{
    std::fill(std::begin(max_sent_per_period_),
              std::end(max_sent_per_period_),
              0);
    std::fill(std::begin(cur_sent_per_period_),
              std::end(cur_sent_per_period_),
              0);
    std::fill(std::begin(dev_period_length_),
              std::end(dev_period_length_),
              0);
    std::fill(std::begin(dev_period_ticks_),
              std::end(dev_period_ticks_),
              0);

    fds_.reserve(if_names.size() + 1);

    // Create internal events pipe, put it first in the list.
    if (pipe(pipe_) != 0) {
        throw std::runtime_error("Cannot create internal events pipe!");
        return;
    }

    fcntl(pipe_[0], F_SETFL, O_NONBLOCK);
    fcntl(pipe_[1], F_SETFL, O_NONBLOCK);
    fds_.push_back(pipe_[0]);

    for (const auto& if_name: if_names) {
        if (if_name.size() > IFNAMSIZ) {
            CAN_LOG_ERROR("Invalid interface name: %s", if_name.c_str());
            continue;
        }

        int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (fd < 0) {
            CAN_LOG_ERROR("Cannot create PF_CAN socket for interface %s.",
                          if_name.c_str());
            continue;
        }

        fcntl(fd, F_SETFL, O_NONBLOCK);

        //Enable error receiving
        can_err_mask_t err_mask = 0x000001FF; //All errors
        setsockopt(fd, SOL_CAN_RAW, CAN_RAW_ERR_FILTER,&err_mask, sizeof(err_mask));

        
        struct ifreq ifr;
        memset(&ifr, 0, sizeof(struct ifreq));
        std::copy(if_name.begin(), if_name.end(), ifr.ifr_name);
        ioctl(fd, SIOCGIFINDEX, &ifr);

        struct sockaddr_can addr;
        memset(&addr, 0, sizeof(struct sockaddr_can));
        addr.can_family  = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(fd, 
                 (struct sockaddr*)(&addr), 
                 sizeof(addr)) < 0) {
            int err = errno;

            CAN_LOG_ERROR("Cannot bind socket on interface %s, "
                          "reason: %s.",                     
                          if_name.c_str(),
                          strerror(err));
            close(fd);
            continue;
        }

        fds_.push_back(fd);

        CAN_LOG_DEBUG("Socket ready on %s.", if_name.c_str());

    }

    can_send_queues_.resize(if_names.size());

    device_queue_map_.fill(-1);

    running_ = true;
    sched_thread_ = std::thread(std::bind(&CANManager::schedLoop, this));
    main_thread_  = std::thread(std::bind(&CANManager::mainLoop,  this));

}

CANManager::~CANManager()
{
    std::cerr<<"~CANManager"<<std::endl;
    stop();

    //Close all fds
    for (auto& i: fds_) { 
        close(i);
    }
    main_thread_.join();
    sched_thread_.join();
 
    std::cerr<<"~CANManager done!"<<std::endl;
}

void CANManager::pushInternalEvent(const char v)
{
    std::unique_lock<MutexType> lock(mutex_);
    int r = write(pipe_[1], &v, sizeof(char));
    if (r != sizeof(char)) {
        CAN_LOG_ERROR("Internal pipe write error.");
    }
}

void CANManager::stop()
{
    CAN_LOG_DEBUG("CANManager::stop()");
    running_ = false;
    pushInternalEvent();
    wait_msgs_cond_.notify_all(); 
    sched_cond_.notify_all();
    CAN_LOG_DEBUG("CANManager::stop() done");
}

bool CANManager::shouldRun()
{
    std::unique_lock<MutexType> lock(mutex_);
    return running_;
}

void CANManager::processFrame(const CANFrame& frame)
{
    // NOTE: We push frames in the queue, and convert them at the processing
    // stage to avoid spending too much time in the CAN thread.
    {
        QueueLock lock(msg_recv_queue_mtx_);
        if (msg_recv_queue_.size() < MAX_MSG_QUEUE_SIZE) {
            CANFramePtr frame_ptr(new CANFrame(frame));
    
            //Look for error frames
            if (frame_ptr->can_id & CAN_ERR_FLAG)
            {
                CAN_LOG_WARN("ERROR FRAME CAUGHT id: %x data %x %x %x %x",
                    frame_ptr->can_id, 
                    frame_ptr->data[0],
                    frame_ptr->data[1],
                    frame_ptr->data[2],
                    frame_ptr->data[3]);
            }
            else
            {
                msg_recv_queue_.push(frame_ptr);
            }
        } else {
            CAN_LOG_WARN("Reception queue full!");
        }
    }
    wait_msgs_cond_.notify_all();
}

bool CANManager::popOneMessage(LaboriusMessage& msg)
{
    // Copy the front frame from the queue (if available) and then transform it.
    CANFramePtr frame;
    {
        QueueLock lock(msg_recv_queue_mtx_);
        if (msg_recv_queue_.empty()) {
            return false;
        }

        frame = msg_recv_queue_.front();
        msg_recv_queue_.pop();
    }

    irl_can_bus::frameToMsg(*frame, msg);
    
    return true;
}

bool CANManager::waitForMessages()
{
    if (running_)
    {
        
        QueueLock lock(msg_recv_queue_mtx_);
        if (!msg_recv_queue_.empty()) {
            return true;
        }
        
        
        std::unique_lock<std::mutex> wait_lock(wait_msgs_mtx_);
    #ifdef CAN_NO_TIMEOUT
        wait_msgs_cond_.wait(wait_lock);
        return true;
    #else
        return wait_msgs_cond_.wait_for(wait_lock, 
                                        std::chrono::seconds(1)) !=
               std::cv_status::timeout;
    #endif
    }
    return false;
}

void CANManager::pushOnCANSendQueue(const CANFramePtr& frame_ptr)
{
    int dev_id = deviceIDFromFrame(*frame_ptr);
    int if_i   = device_queue_map_[dev_id];

    if (if_i < 0) {
        for (int j = 0; j < can_send_queues_.size(); ++j) {
            Queue& cq = can_send_queues_[j];
            cq.push(frame_ptr);
        }
    } else {
        Queue& cq = can_send_queues_[if_i];
        cq.push(frame_ptr);
    }
}

void CANManager::pushOneMessage(const LaboriusMessage& msg)
{
    CANFramePtr frame_ptr(new CANFrame());
    irl_can_bus::msgToFrame(msg, *frame_ptr);
    int dev_id = msg.msg_dest;

    bool should_notify = false;
    {
        QueueLock lock(send_queues_mtx_);
        Queue& q = dev_send_queues_[dev_id];

        should_notify = q.empty();

        if (q.size() < MAX_MSG_QUEUE_SIZE) {
            q.push(frame_ptr);
        } else {
            CAN_LOG_ERROR("Device %i send queue full, dropping message.",
                          dev_id);
        }
    }

    // If a  previously empty queue was found, notify the main loop to wake it up.
    if (should_notify) {
        pushInternalEvent();
    }

    return;
}

void CANManager::mainLoop()
{
    using pollfd = struct pollfd;
    std::vector<pollfd> pollfds(fds_.size());
    for (int i = 0; i < fds_.size(); ++i) {
        pollfd& pfd = pollfds[i];
        pfd.fd = fds_[i];
        pfd.events = POLLPRI | POLLIN | POLLERR | POLLHUP | POLLNVAL;
    }

    while (shouldRun()) {
        // First step: distribute new messages into device queues while
        // respecting throttling.
        {
            QueueLock lock(send_queues_mtx_);
            for (int dev_id = 0; dev_id < dev_send_queues_.size(); ++dev_id) {
                Queue& dq = dev_send_queues_[dev_id];
                while (!dq.empty() && !shouldThrottle(dev_id)) {
                    pushOnCANSendQueue(dq.front());
                    ++(cur_sent_per_period_[dev_id]);
                    dq.pop();
                }

            }
        }

        for (int i = 1; i < fds_.size(); ++i) {
            // Reset event types to poll for, add POLLOUT only if the
            // corresending queue has at least one message.
            pollfd& pfd = pollfds[i];
            pfd.events = POLLPRI | POLLIN | POLLERR | POLLHUP | POLLNVAL;
            {
                QueueLock lock(send_queues_mtx_);
                int if_id = i - 1;
                if (!can_send_queues_[if_id].empty()) {
                    pfd.events |= POLLOUT;
                }
            }
        }

        poll(&pollfds[0], pollfds.size(), -1);

        // Internal events pipe, just flush the buffer if there's anything.
        if (pollfds[0].revents & POLLIN) {

            CAN_LOG_DEBUG("Got internal event.");
            char v;
            while (read(fds_[0], &v, sizeof(v)) == sizeof(v));
        }

        // Process in & out queues.
        const size_t frame_size = sizeof(CANFrame);
        for (int i = 1; i < fds_.size(); ++i) {
            if (pollfds[i].revents & POLLIN) {
                //CAN_LOG_DEBUG("POLLIN on %i.", i);
                CANFrame frame_in;

                while (read(fds_[i], &frame_in, frame_size) == frame_size) {
                    /*
                    CAN_LOG_DEBUG("Got frame from %i, cmd: %i.",
                                  deviceIDFromFrame(frame_in),
                                  deviceCmdFromFrame(frame_in));
                    */
                    processFrame(frame_in);
                }
                // Update the send queue map.
                {
                    QueueLock lock(send_queues_mtx_);
                    // Always store the interface on which we received a message
                    // from a identified device.
                    // Note that fds are 1+ than the queue index.
                    device_queue_map_[deviceIDFromFrame(frame_in)] = i - 1;
                }

            }

            if (pollfds[i].revents & POLLOUT) {
                QueueLock lock(send_queues_mtx_);
                int q_i = i - 1;
                Queue& csq = can_send_queues_[q_i];
                while (!csq.empty()) {
                    const CANFramePtr& frame_out = csq.front();
                    //CAN_LOG_DEBUG("Sending for %i, cmd: %i",
                    //              deviceIDFromFrame(*frame_out),
                    //              deviceCmdFromFrame(*frame_out));
                    int bytes_sent = write(fds_[i],
                                           frame_out.get(),
                                           frame_size);
                    if (!bytes_sent == frame_size) {
                        // Sent errors are kept in the throttled queue and
                        // will be retried later.
                        CAN_LOG_ERROR("Send error for %i on q%i, "
                                      "frame dropped.",
                                      deviceIDFromFrame(*frame_out),
                                      q_i);
                    } 
                    csq.pop();
                }
            }
        }
    };

    CAN_LOG_WARN("CANManager main loop done");
    std::cerr<<"CANManager main loop done"<<std::endl;
}

void CANManager::throttling(int dev_id, const ThrottlingDef& td)
{
    if (!td.valid()) {
        return;
    }

    QueueLock lock(send_queues_mtx_);

    int dpl = td.period_length.count();
    int sp  = sched_period_.count();
    CAN_LOG_DEBUG("throttling for %i: dpl: %i sp: %i",
                  dev_id,
                  dpl,
                  sp);

    if (dpl % sp) {
        CAN_LOG_WARN("The throttling period for %i (%i) is not a multiple "
                     "of the reference period of CANManager (%i).",
                     dev_id,
                     dpl,
                     sp);
    }

    dev_period_length_[dev_id]   = dpl / sp; 
    max_sent_per_period_[dev_id] = td.max_per_period;

    throttled_devs_.push_back(dev_id);

}

void CANManager::throttlingPeriod(SchedTimeBase p)
{
    QueueLock lock(send_queues_mtx_);
    sched_period_ = p;
}

bool CANManager::shouldThrottle(int dev_id) const
{
    int m = max_sent_per_period_[dev_id];
    if (m > 0) {
        int c = cur_sent_per_period_[dev_id];
        return (c >= m);
    } else {
        return false;
    }
}

void CANManager::schedLoop()
{
    while (shouldRun())
    {
        SchedTimePoint start = SchedClock::now();
        bool should_notify = false;

        // Update the device ticks and reset counts at the end of a period.
        {
            QueueLock lock(send_queues_mtx_);
            for (auto i: throttled_devs_) {
                int        m     = max_sent_per_period_[i];
                int&       ticks = dev_period_ticks_[i];
                const int& dev_p = dev_period_length_[i];
                // Resetting count on tick 0.
                if (ticks == 0) {
                    int& c = cur_sent_per_period_[i];
                    //if (c > 0) {
                    //    CAN_LOG_DEBUG("cycle for %i, c: %i", i, c);
                    //}
                    should_notify = true;
                    c = 0;
                }
                ticks = (ticks + 1) % dev_p;
            }

        }

        // If any count was at max (indicating throttling), notify the main 
        // loop.
        if (should_notify) {
            pushInternalEvent();
        }

        // Sleep for the rest of the period.
        auto dur = SchedClock::now() - start;
        auto rem = sched_period_ - dur;
        if (rem.count() > 0) {
            std::this_thread::sleep_for(rem);
        }
    }

    CAN_LOG_WARN("CANManager sched loop done");
    std::cerr<<"CANManager sched loop done"<<std::endl;
}

void CANManager::requestMem(unsigned int device_id,
                            unsigned int offset,
                            unsigned int size,
                            unsigned int priority)
{
    //CAN_LOG_DEBUG("Mem request for %i, offset %i", device_id, offset);
    LaboriusMessage msg;
    irl_can_bus::requestMem(msg, device_id, offset, size, priority);
    pushOneMessage(msg);
}

void CANManager::writeMem(unsigned int device_id,
                            unsigned int offset,
                            unsigned char* data,
                            unsigned int size,
                            unsigned int priority)
{
    //CAN_LOG_DEBUG("Mem request for %i, offset %i", device_id, offset);
    LaboriusMessage msg;
    irl_can_bus::writeMem(msg, device_id, offset, data, size, priority);
    pushOneMessage(msg);
}

