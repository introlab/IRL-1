#ifndef LABORIUS_MESSAGE_HPP
#define LABORIUS_MESSAGE_HPP

#include <string.h>

namespace irl_can_bus
{

/**
     \brief LABORIUS_MESSAGE
     
     This data structure holds CAN message informations
     to be written to the CAN bus or read from the
     CAN bus. The 29 bit extended frames are separated into
     LABORIUS home made protocol and fields.
     
     \authors $Author: dominic $
     \version $Revision: 1.1 $
     \date $Date: 2007-01-11 19:21:20 $
*/
struct LaboriusMessage 
{
     /** Default Constructor */
     LaboriusMessage()
     {
          msg_priority = 0;
          msg_type = 0;
          msg_cmd = 0;
          msg_boot = 0;
          msg_dest = 0;
          msg_data_length =0;
          msg_remote = 0;
          memset(msg_data,0,8);
          msg_filter_hit=-1;
          msg_dwTime = 0;               
     }
     /** Copy Constructor
          \param message The message to copy
     */
     LaboriusMessage(const LaboriusMessage &message) {    
          msg_priority=message.msg_priority;
          msg_type=message.msg_type;
          msg_cmd=message.msg_cmd;
          msg_boot=message.msg_boot;
          msg_dest=message.msg_dest;
          msg_data_length=message.msg_data_length;
          memcpy(msg_data,message.msg_data,8);
          msg_remote=message.msg_remote;
          msg_filter_hit=message.msg_filter_hit;
          msg_dwTime = message.msg_dwTime;
     }
     /// Message priority 0=high to 7=low
     unsigned char msg_priority; 
     /// Message type (8 bit mask)
     unsigned char msg_type;
     ///Message command 0 to 255   
     unsigned char msg_cmd;
     ///Message boot bits used to program PIC devices 0 to 3 (boot bits)
     unsigned char msg_boot;
     ///Message destination 0 to 255, 255 = broadcast
     unsigned char msg_dest;
     ///Message data length 0 to 8 bytes
     char msg_data_length;
     ///Message data maximum 8 bytes
     unsigned char msg_data[8];
     ///Message remote request bit (RTR)
     unsigned char msg_remote;
     ///Filter hit (used by the PCANDevice driver)
     int msg_filter_hit;
     ///Time stamp (ms)
     unsigned int msg_dwTime;
     
     
};

}

#endif

