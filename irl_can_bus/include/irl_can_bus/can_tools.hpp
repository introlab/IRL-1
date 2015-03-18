#ifndef CAN_TOOLS_HPP
#define CAN_TOOLS_HPP

#include <linux/can.h>
#include "laborius_message.hpp"
#include <cstdio>
#include <iostream>
#include <functional>

namespace irl_can_bus
{
    static const int MAX_CAN_DEV_ID    = 255;
    static const int MAX_CAN_DEV_COUNT = MAX_CAN_DEV_ID + 1;

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

    /// \brief Return the command byte from a reference to a full CAN frame.
    static int deviceCmdFromFrame(const CANFrame& f)
    {
        return (f.can_id & 0xFF00) >> 8;
    }

    /// \brief Build a memory read request message.
    ///
    /// \param msg       A reference to the message to be built.
    /// \param device_id CAN device id (0-255)
    /// \param offset    Memory offset where to start reading.
    /// \param size      Size to read, cannot be more than 8.
    /// \param priority  CAN frame priority.
    void requestMem(LaboriusMessage& msg,
                    unsigned int     device_id,
                    unsigned int     offset,
                    unsigned int     size, 
                    unsigned char    priority = 0);

    /// \brief Build a memory write message.
    ///
    /// \param msg       A reference to the message to be built.
    /// \param device_id CAN device id (0-255)
    /// \param offset    Memory offset where to start reading.
    /// \param data      Data to write
    /// \param size      Size to write, cannot be more than 8.
    /// \param priority  CAN frame priority.
    void writeMem(LaboriusMessage& msg,
                    unsigned int     device_id,
                    unsigned int     offset,
                    unsigned char*   data,
                    unsigned int     size, 
                    unsigned char    priority = 0);


    namespace log
    {
        enum LogID
        {
            CAN_LOG_DEBUG = 0,
            CAN_LOG_WARN,
            CAN_LOG_ERROR
        };

        using LoggerFunction = std::function<void (LogID, const char*)>;

        static const char * logName(LogID id)
        {
            switch (id) {
                case CAN_LOG_DEBUG:
                    return "DEBUG";
                    break;
                case CAN_LOG_WARN:
                    return "WARNING";
                    break;
                case CAN_LOG_ERROR:
                    return "ERROR";
                    break;
                default:
                    return "DEFAULT";
                    break;
            };
        }

        void            logLineFormat(LogID id, const char* format, ...);
        const LoggerFunction& loggerFunction();
        void            loggerFunction(const LoggerFunction& fun);
        void            loggerFunctionDefault(LogID id, const char* str);
    }
}

#ifdef DEBUG
#define CAN_LOG_DEBUG(...) irl_can_bus::log::logLineFormat(irl_can_bus::log::CAN_LOG_DEBUG, __VA_ARGS__);
#else
#define CAN_LOG_DEBUG(...)
#endif

#define CAN_LOG_WARN(...) irl_can_bus::log::logLineFormat(irl_can_bus::log::CAN_LOG_WARN, __VA_ARGS__);
#define CAN_LOG_ERROR(...) irl_can_bus::log::logLineFormat(irl_can_bus::log::CAN_LOG_ERROR, __VA_ARGS__);

#endif

