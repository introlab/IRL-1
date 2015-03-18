#include <irl_can_bus/can_tools.hpp>
#include <irl_can_bus/can_base_macros.h>

#include <algorithm>
#include <cstdarg>
#include <cassert>

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

void irl_can_bus::requestMem(LaboriusMessage& msg,
                             unsigned int     device_id,
                             unsigned int     offset,
                             unsigned int     size, 
                             unsigned char    priority)
{
    assert(size <= 8);

    msg.msg_priority = priority;
    msg.msg_type = CAN_TYPE_REQUEST_DATA;
    msg.msg_cmd = offset;
    msg.msg_dest = device_id;
    msg.msg_boot = (CAN_REQUEST_RAM << 1) | (CAN_REQUEST_READ);

    msg.msg_remote = 1;
    msg.msg_data_length = size;
}

void irl_can_bus::writeMem(LaboriusMessage& msg,
                             unsigned int     device_id,
                             unsigned int     offset,
                             unsigned char*   data,
                             unsigned int     size, 
                             unsigned char    priority)
{
    assert(size <= 8);

    msg.msg_priority = priority;
    msg.msg_type = CAN_TYPE_REQUEST_DATA;
    msg.msg_cmd = offset;
    msg.msg_dest = device_id;
    msg.msg_boot = (CAN_REQUEST_RAM << 1) | (CAN_REQUEST_WRITE);

    msg.msg_remote = 0;
    msg.msg_data_length = size;

    //copy data
    memcpy(msg.msg_data,data,size);
}


void irl_can_bus::log::logLineFormat(LogID id, const char* format, ...)
{
    char buffer[4096];
    int size = 0;
    
    size = snprintf(buffer, sizeof(buffer), "CAN %s: ", logName(id));

    va_list vargs;
    va_start(vargs, format);
    size += vsnprintf(&buffer[size], sizeof(buffer), format, vargs);
    va_end(vargs);

    snprintf(&buffer[size], sizeof(buffer), "\n");

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

