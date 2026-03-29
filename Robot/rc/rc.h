#ifndef MPU6050_RC_H
#define MPU6050_RC_H

#include "stm32h7xx.h"
#include <cstdint>
#include <sys/cdefs.h>
#include "pyro_core_dma_heap.h"
#include "pyro_core_def.h"

using namespace pyro;

#define RC_CH_VALUE_MIN ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)

#define KEY_PRESSED_OFFSET_W ((uint16_t)0x01 << 0)
#define KEY_PRESSED_OFFSET_S ((uint16_t)0x01 << 1)
#define KEY_PRESSED_OFFSET_A ((uint16_t)0x01 << 2)
#define KEY_PRESSED_OFFSET_D ((uint16_t)0x01 << 3)
#define KEY_PRESSED_OFFSET_Q ((uint16_t)0x01 << 4)
#define KEY_PRESSED_OFFSET_E ((uint16_t)0x01 << 5)
#define KEY_PRESSED_OFFSET_Shift ((uint16_t)0x01 << 6)
#define KEY_PRESSED_OFFSET_Ctrl ((unt16_t)0x01 << 7)

#pragma pack(push,1)
typedef struct
{
    struct
    {
        uint16_t ch0;
        uint16_t ch1;
        uint16_t ch2;
        uint16_t ch3;
        uint8_t s1;
        uint8_t s2;
    }rc;

    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    }mouse;

    struct
    {
        uint16_t v;
    }key;
}Rc_ctrl_t;

enum sw_state_t
{
    SW_UP = 1,
    SW_DOWN = 2,
    SW_LEFT = 3,
};

enum sw_ctrl_t
{
    SW_UP_TO_MID   = 1,
    SW_MID_TO_DOWN = 2,
    SW_DOWN_TO_MID = 3,
    SW_MID_TO_UP   = 4,
};

typedef struct
{
    sw_ctrl_t sw_ctrl;
    sw_state_t sw_state;
    float time;
}switch_t;

enum mouse_state_t
{
    OFF = 0,
    ON = 1,
};

typedef struct
{
    mouse_state_t mouse_state;
    float press_time;
}press_t;
#pragma pack(pop)

typedef struct
{
    struct
    {
        float ch0;
        float ch1;
        float ch2;
        float ch3;

        switch_t s1;
        switch_t s2;
    }rc;

    struct
    {
        float x;
        float y;
        float z;
        press_t press_l;
        press_t press_r;
    }mouse;
}dr16_ctrl_t;

class rc_ctrl_t
{
public:
    rc_ctrl_t(Rc_ctrl_t *data);
    ~rc_ctrl_t();

    void init();
    void unpack_data(uint8_t *data);
    void control_logic(dr16_ctrl_t *dr16_data) const;
    void handle_rx_event(uint16_t size);
    static void uart_rx_wrapper(UART_HandleTypeDef *huart,uint16_t Size);
private:
    dr16_ctrl_t dr_data{};
    Rc_ctrl_t *rc_data{};
    uint8_t rx_buf_switch{};
    uint8_t *rx_buf[2]{};
    uint8_t UART5_MAX_RECV_LEN = 18;

    static rc_ctrl_t* instance;
};


#endif //MPU6050_RC_H