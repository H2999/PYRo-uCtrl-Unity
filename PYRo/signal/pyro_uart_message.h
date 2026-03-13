// #ifndef __PYRO_PYRO_UART_MESSAGE_H__
// #define __PYRO_PYRO_UART_MESSAGE_H__
//
// #include <cstdint>
//
// struct frame_header
// {
//     uint8_t sof;
// } __attribute__((packed));
//
// struct frame_tailer
// {
//     uint16_t crc16;
// } __attribute__((packed));
//
// typedef struct
// {
//     float shoot_yaw;
//     float shoot_yaw_speed;
//     float shoot_yaw_acceleration;
//     float shoot_pitch;
//     float shoot_pitch_speed;
//     float shoot_pitch_acceleration;
//     uint8_t fire           : 1;
//     uint8_t is_single_shot : 1;
//     uint8_t target_id      : 6;
//     uint8_t aim_state;
// } __attribute__((packed)) aim2mcu_data_t;
//
// typedef struct
// {
//     float curr_yaw;
//     float curr_pitch;
//     float self_v_magnitude;
//     float self_v_angle;
//     uint8_t shoot_delay;
//     uint8_t state       : 5;
//     uint8_t stop_record : 1;
//     uint8_t enemy_color : 1;
// } __attribute__((packed)) mcu2aim_data_t;
//
// typedef struct
// {
//     uint16_t stop_record;
//     uint16_t enemy_color;
// } __attribute__((packed)) mcu2nav_data_t;
//
// typedef struct
// {
//     frame_header header;
//     aim2mcu_data_t data;
//     frame_tailer tailer;
// } __attribute__((packed)) aim2mcu_msg_t;
//
// typedef struct
// {
//     frame_header header;
//     mcu2aim_data_t data;
//     frame_tailer tailer;
// } __attribute__((packed)) mcu2aim_msg_t;
//
// typedef struct
// {
//     frame_header header;
//     mcu2nav_data_t data;
//     frame_tailer tailer;
// } __attribute__((packed)) mcu2nav_msg_t;
//
// #endif


#ifndef PYRO_STRUCT_H
#define PYRO_STRUCT_H


#include <cstdint>

#pragma pack(push, 1)

struct FrameHeader
{
    uint8_t sof;
};

struct FrameTailer
{
    uint16_t crc16;
};

struct OutputData
{
    float curr_yaw;
    float curr_pitch;
    uint8_t state;
    uint8_t autoaim;
    uint8_t enemy_color;
    float curr_speed;
    uint16_t shoot_delay;
};

struct InputData
{
    uint8_t fire;
    float shoot_yaw;
    float shoot_pitch;
    float avg_speed;
    uint8_t food;
};

struct StateBytes
{
    FrameHeader frame_header;
    InputData input_data;
    FrameTailer frame_tailer;
};

struct OperateBytes
{
    FrameHeader frame_header;
    OutputData output_data;
    FrameTailer frame_tailer;
};

#pragma pack(pop)

#endif // PYRO_STRUCT_H

