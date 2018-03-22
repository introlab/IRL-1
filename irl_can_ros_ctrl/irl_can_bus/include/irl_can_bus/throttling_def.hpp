#ifndef THROTTLING_DEF_HPP
#define THROTTLING_DEF_HPP

namespace irl_can_bus
{
    using TimeBase = std::chrono::microseconds;

    /// \brief A simple structure defining a device throttling characteristics.
    struct ThrottlingDef
    {
        /// \brief Throttling reference period, in us.
        TimeBase period_length;
        /// \brief Maximum messages sent per throttling period.
        int max_per_period;

        /// \brief Return true if this device should be throttled.
        bool valid() const
        {
            return (period_length.count() > 0) && (max_per_period > 0);
        }
            
    };
}

#endif
