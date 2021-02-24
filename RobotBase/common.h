//
// Created by jsz on 5/15/20.
//

#ifndef ROBOTBASE_COMMON_H
#define ROBOTBASE_COMMON_H




struct _OtherParam {
    // color of our team: 0 -> blue; 1 -> red

    uint8_t color = UNKNOWN;
    uint8_t mode = AUTOAIM;
    uint8_t level = 0;
    uint8_t id = 0;
};
/*
 * contains information about our team color, mode of aiming, s.
 */
typedef _OtherParam OtherParam;
#endif //ROBOTBASE_COMMON_H
