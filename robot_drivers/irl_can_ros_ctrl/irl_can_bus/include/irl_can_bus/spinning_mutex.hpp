#ifndef SPINNING_MUTEX_HPP
#define SPINNING_MUTEX_HPP

#include <atomic>

namespace irl_can_bus
{
    /// \brief  A mutex with spinning wait lock.
    ///
    /// Follows BasicLockable requirements for std::lock_guard.
    class SpinningMutex
    {
    private:
        std::atomic<bool> lock_;

    public:
        SpinningMutex(): lock_(false) {};

        /// \brief Locks the mutex.
        ///
        /// Blocking until locked.
        void lock()
        {
            while (lock_.exchange(true)); // Spinning wait.
        }

        /// \brief Unlock the mutex.
        ///
        /// Unblocking.
        void unlock()
        {
            lock_.store(false);
        }

    };

}

#endif

