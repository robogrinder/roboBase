//
// Created by jsz on 3/8/20.
//

#ifndef SERIAL_PORT_MESSAGE_H
#define SERIAL_PORT_MESSAGE_H

#include <stdint.h>
#include <cstdint>


// TODO: remove paddings
struct serial_gimbal_data
{
    uint8_t rawData[20];
    const int head = 0xaf;
    const int id = 1;
    // how many elements of rawData are valid
    int size;
};

/**
 * provides buffer for receiving data and constants
 */
struct serial_recive_data
{
    char rawData[10];
    const int head = 0xaf;
    int id;
    int size;
};


/**
 * receiveData is translated into input_data
 */
struct Input_data
{
    uint8_t cmdID;
    uint8_t _level;
    uint8_t dbusInfo;
    uint8_t roboID;
    // size does not include the header and the checksum (only count useful information)
    const uint8_t size = 4;
};

// =======================
// below enums are for input/output enum data types
typedef enum _color {
    BLUE, RED, UNKNOWN
} Robot_color;

typedef enum _mode{
    BIGBUFF, AUTOAIM, AUTOFIRE
} mode;


typedef enum _RobotID {
    INFANTRY, HERO, SENTRY, DRONE
} RobotID;
#endif //SERIAL_PORT_MESSAGE_H

