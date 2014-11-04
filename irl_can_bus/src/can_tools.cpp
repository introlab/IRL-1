#include <irl_can_bus/can_tools.hpp>

#include <algorithm>
#include <cstdarg>

namespace {
    static irl_can_bus::log::LoggerFunction logger_fun_ =
        &irl_can_bus::log::loggerFunctionDefault;
}

using namespace irl_can_bus;

void irl_can_bus::frameToMsg(const CANFrame& frame, LaboriusMessage& msg)
{
    msg.msg_priority    = (frame.can_id & 0x1C000000) >> 26;
    msg.msg_type        = (frame.can_id & 0x03FC0000) >> 18;
    msg.msg_boot        = (frame.can_id & 0x00030000) >> 16;
    msg.msg_cmd         = (frame.can_id & 0x0000FF00) >> 8;
    msg.msg_dest        = (frame.can_id & 0x000000FF);

    int size = std::min(8, (int)(frame.can_dlc));
    msg.msg_data_length = size;
    std::copy(&frame.data[0], &frame.data[size], &msg.msg_data[0]);

    if (frame.can_id & CAN_RTR_FLAG)
        msg.msg_remote = 1;
}

void irl_can_bus::msgToFrame(const LaboriusMessage& msg, CANFrame& frame)
{
    frame.can_id  = ((unsigned int)(msg.msg_priority)   << 26)  & 0x1C000000;
    frame.can_id |= ((unsigned int)(msg.msg_type)       << 18)  & 0x03FC0000;
    frame.can_id |= ((unsigned int)(msg.msg_boot)       << 16)  & 0x00030000;
    frame.can_id |= ((unsigned int)(msg.msg_cmd)        <<  8)  & 0x0000FF00;
    frame.can_id |= ((unsigned int)(msg.msg_dest))              & 0x000000FF;

    if (msg.msg_remote)
        frame.can_id |= CAN_RTR_FLAG;

    // Extended frame mode.
    frame.can_id |= CAN_EFF_FLAG;

    // Data copy.
    int size = std::min(8, (int)(msg.msg_data_length));
    frame.can_dlc = size;
    std::copy(&msg.msg_data[0], &msg.msg_data[size], &frame.data[0]);

}

void irl_can_bus::log::logLineFormat(LogID id, const char* format, ...)
{
    static char buffer[4096];
    
    int prefix_size = snprintf(buffer, sizeof(buffer), "CAN %s: ", logName(id));

    va_list vargs;
    va_start(vargs, format);
    vsnprintf(&buffer[prefix_size], sizeof(buffer), format, vargs);
    va_end(vargs);

    loggerFunction()(id, buffer);
}

const irl_can_bus::log::LoggerFunction& irl_can_bus::log::loggerFunction()
{
    return ::logger_fun_;
}

void irl_can_bus::log::loggerFunction(const LoggerFunction& fun)
{
    ::logger_fun_ = fun;
}

void irl_can_bus::log::loggerFunctionDefault(LogID id, const char* str)
{
    std::cerr << str;
}

