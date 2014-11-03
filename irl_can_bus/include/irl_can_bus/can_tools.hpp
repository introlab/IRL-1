#ifndef CAN_TOOLS_HPP
#define CAN_TOOLS_HPP

#include <linux/can.h>
#include "laborius_message.hpp"
#include <cstdio>
#include <iostream>

namespace irl_can_bus
{
    static const int MAX_CAN_DEV_ID = 255;

    using CANFrame  = struct can_frame;

    /// \brief Convert a CAN frame to a LaboriusMessage structure.
    void frameToMsg(const CANFrame& f, LaboriusMessage& msg);

    /// \brief Convert a LaboriusMessage to a CAN frame structure.
    void msgToFrame(const LaboriusMessage& msg, CANFrame& frame);

    /// \brief Return the device ID from a reference to a full CAN frame.
    static int deviceIDFromFrame(const CANFrame& f)
    {
        return f.can_id & 0xFF;
    }

    enum LogID
    {
        CAN_LOG_DEBUG = 0
    };

    static std::ostream& logStream(LogID id)
    {
        return std::cerr;
    }

    namespace log
    {
        static FILE* debug_file_ = stderr;
        static void setFile(LogID id, FILE* f)
        {
            switch (id) {
                case CAN_LOG_DEBUG:
                    debug_file_ = f;
                break;

                default:
                break;
            }
        }
    }

    static FILE* logFile(LogID id)
    {
        if (id == CAN_LOG_DEBUG) {
            return log::debug_file_;
        } else {
            return stderr;
        }
    }

    static const char * logName(LogID id)
    {
        if (id == CAN_LOG_DEBUG) {
            return "DEBUG";
        } else {
            return "DEFAULT";
        }
    }

    void logLine(LogID id, const char* format, ...);

}

#ifdef DEBUG
#define CAN_LOG_DEBUG(...)   irl_can_bus::logLine(CAN_LOG_DEBUG, __VA_ARGS__);
#else
#define CAN_LOG_DEBUG(...)
#endif

#endif

