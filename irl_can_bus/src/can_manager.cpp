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

using namespace irl_can_bus;

CANManager::CANManager(const std::vector<std::string> if_names): 
    running_(false),
    sched_period_(100)
{
    fds_.reserve(if_names.size() + 1);

    // Create internal events pipe, put it first in the list.
    if (pipe(pipe_) != 0) {
        throw std::runtime_error("Cannot create internal events pipe!");
        return;
    }

    fcntl(pipe_[0], F_SETFL, O_NONBLOCK);
    fcntl(pipe_[1], F_SETFL, O_NONBLOCK);
    fds_.push_back(pipe_[0]);

    for (auto i = if_names.cbegin(); i != if_names.cend(); ++i) {
        const std::string if_name = *i;
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

    msg_send_queues_.resize(if_names.size());
    device_queue_map_.fill(-1);

    running_ = true;
    sched_thread_ = std::thread(std::bind(&CANManager::schedLoop, this));
    main_thread_  = std::thread(std::bind(&CANManager::mainLoop,  this));

}

CANManager::~CANManager()
{
    stop();
    main_thread_.join();
    for (auto i = fds_.begin(); i != fds_.end(); ++i) {
        close(*i);
    }
}

void CANManager::pushInternalEvent(const char v)
{
    std::unique_lock<MutexType> lock(mutex_);
    write(pipe_[1], &v, sizeof(char));
}

void CANManager::stop()
{
    CAN_LOG_DEBUG("CANManager::stop()");
    pushInternalEvent();
    wait_msgs_cond_.notify_all(); 
    running_ = false;
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
            msg_recv_queue_.push(frame);

        }
    }
    wait_msgs_cond_.notify_one();
}

bool CANManager::popOneMessage(LaboriusMessage& msg)
{
    // Copy the front frame from the queue (if available) and then transform it.
    CANFrame frame;
    {
        QueueLock lock(msg_recv_queue_mtx_);
        if (msg_recv_queue_.empty()) {
            return false;
        }

        frame = msg_recv_queue_.front();
        msg_recv_queue_.pop();
    }

    irl_can_bus::frameToMsg(frame, msg);
    
    return true;
}

void CANManager::waitForMessages()
{
    {
        QueueLock lock(msg_recv_queue_mtx_);
        if (!msg_recv_queue_.empty()) {
            return;
        }
    }
    
    std::unique_lock<std::mutex> wait_lock(wait_msgs_mtx_);
    wait_msgs_cond_.wait(wait_lock);
}

void CANManager::pushOnQueue(const CANFrame& frame, int q)
{
    QueueLock lock(msg_send_queues_mtx_);

    assert(q < msg_send_queues_.size());

    bool notify = false;

    if (msg_send_queues_[q].size() < MAX_MSG_QUEUE_SIZE) {
        CAN_LOG_DEBUG("Pushing message for %i on q%i.",
                      deviceIDFromFrame(frame),
                      q);
        notify = msg_send_queues_[q].empty();
        msg_send_queues_[q].push(frame);
    }

    if (notify) {
        // Wake up the loop if the send queue was previously empty.
        CAN_LOG_DEBUG("Empty send queue, notify internal queue.");
        pushInternalEvent();
    }
}

void CANManager::pushOneMessage(const LaboriusMessage& msg)
{
    CANFrame frame;
    irl_can_bus::msgToFrame(msg, frame);

    // Check if we know on which interface the device is.
    // If it isn't known, broadcast on all interfaces.
    int queue_i = device_queue_map_[msg.msg_dest];
    if (queue_i < 0) {
        for (int i = 0; i < msg_send_queues_.size(); ++i) {
            pushOnQueue(frame, i);
        }
    } else {
        pushOnQueue(frame, queue_i);
    }

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
        for (int i = 1; i < fds_.size(); ++i) {
            // Reset event types to poll for, add POLLOUT only if the
            // corresending queue has at least one message.
            pollfd& pfd = pollfds[i];
            pfd.events = POLLPRI | POLLIN | POLLERR | POLLHUP | POLLNVAL;
            {
                QueueLock lock(msg_send_queues_mtx_);
                if (!msg_send_queues_[i - 1].empty()) {
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
        CANFrame     frame;
        const size_t frame_size = sizeof(frame);
        for (int i = 1; i < fds_.size(); ++i) {
            if (pollfds[i].revents & POLLIN) {
                while (read(fds_[i], &frame, frame_size) == frame_size) {
                    processFrame(frame);
                }
                // Update the send queue map.
                {
                    QueueLock lock(msg_send_queues_mtx_);
                    // Always store the interface on which we received a message
                    // from a identified device.
                    // Note that fds are 1+ than the queue index.
                    device_queue_map_[deviceIDFromFrame(frame)] = i - 1;
                }

            }
            if (pollfds[i].revents & POLLOUT) {
                QueueLock lock(msg_send_queues_mtx_);
                std::queue<CANFrame> throttled_queue;
                int q_i = i - 1;
                while (!msg_send_queues_[q_i].empty()) {
                    int dev_id = deviceIDFromFrame(frame);
                    if (shouldThrottle(dev_id)) {
                        throttled_queue.push(frame);
                    } else {
                        CAN_LOG_DEBUG("Sending for %i on q%i, cmd: %i.",
                            dev_id,
                            q_i,
                            deviceCmdFromFrame(frame));
                        const CANFrame& frame = msg_send_queues_[q_i].front();
                        if (write(fds_[i], &frame, frame_size) == frame_size) {
                            ++cur_sent_per_period_[dev_id];
                        }
                            // Sent errors are kept in the throttled queue and
                            // will be retried later.
                            CAN_LOG_ERROR("Frame send error for %i on q%i, "
                                          "will retry on next cycle.",
                                          dev_id, q_i);
                            throttled_queue.push(frame);
                        } 
                    msg_send_queues_[q_i].pop();
                }

                // Push back the throttled frames on the main queue.
                while (!throttled_queue.empty()) {
                    msg_send_queues_[q_i].push(throttled_queue.front());
                    throttled_queue.pop();
                }
            }
        }
    };
}

void CANManager::throttling(int dev_id, int count)
{
    QueueLock lock(msg_send_queues_mtx_);
    max_sent_per_period_[dev_id] = count;
}

void CANManager::throttlingPeriod(SchedTimeBase p)
{
    QueueLock lock(msg_send_queues_mtx_);
    sched_period_ = p;
}

bool CANManager::shouldThrottle(int dev_id) const
{
    int m = max_sent_per_period_[dev_id];
    if (m > 0) {
        return cur_sent_per_period_[dev_id] >= m;
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

        // Reset the sent counts.
        {
            QueueLock lock(msg_send_queues_mtx_);
            for (int i = 0; i < MAX_CAN_DEV_COUNT; ++i) {
                int m = max_sent_per_period_[i];
                if (m > 0) {
                    int& c = cur_sent_per_period_[i];
                    if (!should_notify && c >= m) {
                        should_notify = true;
                    }
                    c = 0;
                }
            }

            // Also check for non-empty queues.
            if (!should_notify) {
                for (const auto& q: msg_send_queues_) {
                    if (!q.empty()) {
                        should_notify = true;
                        break;
                    }
                }
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
}

void CANManager::requestMem(unsigned int device_id,
                            unsigned int offset,
                            unsigned int size,
                            unsigned int priority)
{
    CAN_LOG_DEBUG("Mem request for %i, offset %i", device_id, offset);
    LaboriusMessage msg;
    irl_can_bus::requestMem(msg, device_id, offset, size, priority);
    pushOneMessage(msg);
}
