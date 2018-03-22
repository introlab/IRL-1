#ifndef CAN_BASE_MACROS_H
#define CAN_BASE_MACROS_H

#define BOOT_LOADER_ADDRESS 0x00009000

#define CMD_NOP 0x00
#define CMD_RESET 0x01
#define CMD_START_WRITE 0x02
#define CMD_END_WRITE 0x03

#define MODE_WRT_UNLCK 0x01;
#define MODE_ERASE_ONLY 0x02;
#define MODE_AUTO_ERASE 0x04;
#define MODE_AUTO_INC 0x08;
#define MODE_ACK 0x10;

/*
MESSAGE PRIORITY DEFINITIONS
*/
#define CAN_PRIORITY_HIGHEST     0
#define CAN_PRIORITY_HIGH        1
#define CAN_PRIORITY_MEDIUM_HIGH 2
#define CAN_PRIORITY_MEDIUM      3
#define CAN_PRIORITY_MEDIUM_LOW  4
#define CAN_PRIORITY_LOW         5
#define CAN_PRIORITY_LOWEST      6
#define CAN_PRIORITY_EVENTS      7

/*
MESSAGE TYPE DEFINITIONS
*/
#define CAN_TYPE_EMERGENCY                0x01
#define CAN_TYPE_ACTUATOR_HIGH_PRIORITY   0x02
#define CAN_TYPE_SENSOR_HIGH_PRIORITY     0x04
#define CAN_TYPE_ACTUATOR_LOW_PRIORITY    0x08
#define CAN_TYPE_SENSOR_LOW_PRIORITY      0x10
#define CAN_TYPE_REQUEST_DATA             0x20
#define CAN_TYPE_USER2                    0x40
#define CAN_TYPE_EVENTS                   0x80
#define NOTHING                           0xFF

/*
EMERGENCY MESSAGE COMMANDS (TYPE_EMERGENCY)
*/
#define CAN_EMERGENCY_CMD_RESET        0x00
#define CAN_EMERGENCY_CMD_STOP         0x01
#define CAN_EMERGENCY_CMD_DISCONNECT   0x02
#define CAN_EMERGENCY_CMD_RECONNECT    0x03
#define CAN_EMERGENCY_CMD_PROGRAM      0x04

#define CAN_REQUEST_READ                0x01
#define CAN_REQUEST_WRITE               0x00

#define CAN_REQUEST_EEPROM              0x01
#define CAN_REQUEST_RAM                 0x00

/*
EVENTS MESSAGE COMMANDS (TYPE EVENTS)
*/
#define CAN_EVENTS_CMD_ALIVE           0x00
#define CAN_EVENTS_CMD_REGISTER        0x01  // unused
#define CAN_EVENTS_CMD_SILENCE         0x02

/*
CAN ADDRESS DEFINITIONS
*/
#define CAN_ADDRESS_BROADCAST 0xFF


// DATA_FLOW_TABLE Stuff
// ADDR, OFFSET, LIMITS on memory
#define DATA_FLOW_ENTRY             0x08
#define MAX_RAM_DATA_ENTRY          0x80
#define MAX_EEPROM_DATA_ENTRY       0x80
#define BIT_SET_LIMITS              0x07
#define SECTION_MASK                0x0F
#define SUB_SECTION_MASK            0xF0
#define SUB_SECTION_SHIFT           0x04
#define NUM_SECTION                   16

//READ_WRITE MEMORY
#define READ_WRITE_1_BYTE           0x08
#define READ_WRITE_2_BYTE           0x09
#define READ_WRITE_3_BYTE           0x0A
#define READ_WRITE_4_BYTE           0x0B
#define READ_WRITE_5_BYTE           0x0C
#define READ_WRITE_6_BYTE           0x0D
#define READ_WRITE_7_BYTE           0x0E
#define READ_WRITE_8_BYTE           0x0F

#define READ_WRITE_BIT_0            0x00
#define READ_WRITE_BIT_1            0x01
#define READ_WRITE_BIT_2            0x02
#define READ_WRITE_BIT_3            0x03
#define READ_WRITE_BIT_4            0x04
#define READ_WRITE_BIT_5            0x05
#define READ_WRITE_BIT_6            0x06
#define READ_WRITE_BIT_7            0x07

/////////////////////////////////////////////////////////////////
//    MEMORY ORGANISATION
//
//    SECTION_ID                       INDEX       MEANING
//    -----------------------------------------------------------
//    CAN_REQUEST_DATA_FLOW_00         0           TABLE VERSION
//    CAN_REQUEST_DATA_FLOW_00         1           PROJECT ID
//    CAN_REQUEST_DATA_FLOW_00         2           GENERAL ID
//    CAN_REQUEST_DATA_FLOW_00         3           MODULE  ID
//    CAN_REQUEST_DATA_FLOW_00         4           CODE VERSION
//    CAN_REQUEST_DATA_FLOW_00         5           STATE OF CONTROLER
//
/////////////////////////////////////////////////////////////////

//MODE ID USED when TYPE = CAN_TYPE_EVENTS and cmd
#define CAN_NORMAL_MODE_ID          0x05
#define CAN_BOOT_MODE_ID            0x0A
#define CAN_IDLE_MODE_ID            0x0B

#endif

