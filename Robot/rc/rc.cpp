#include "rc.h"

#include <cstring>
#include "usart.h"

using namespace pyro;

rc_ctrl_t::rc_ctrl_t(const Rc_ctrl_t *data)
{
    rx_buf[0] = static_cast<uint8_t *>(pvPortDmaMalloc(UART5_MAX_RECV_LEN));
    rx_buf[1] = static_cast<uint8_t *>(pvPortDmaMalloc(UART5_MAX_RECV_LEN));
    rc_data = *data;
}

rc_ctrl_t::~rc_ctrl_t()
{
    vPortDmaFree(rx_buf[0]);
    vPortDmaFree(rx_buf[1]);
}

void rc_ctrl_t::init()
{
    __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx_buf[rx_buf_switch], UART5_MAX_RECV_LEN);
    __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
}

void rc_ctrl_t::unpack_data(uint8_t *data)
{
    rc_data.rc.ch0 = (static_cast<int16_t>(data[0]) | (static_cast<int16_t>(data[1]) << 8)) & 0x07FF;
    rc_data.rc.ch1 = (static_cast<int16_t>(data[1]) >> 3 | (static_cast<int16_t>(data[2]) << 5)) & 0x07FF;
    rc_data.rc.ch2 = (static_cast<int16_t>(data[2]) >> 6 | (static_cast<int16_t>(data[3]) << 2) | (static_cast<int16_t>(data[4]) << 10)) & 0x07FF;
    rc_data.rc.ch3 = (static_cast<int16_t>(data[4]) >> 1 | (static_cast<int16_t>(data[5]) << 7)) & 0x07FF;

    rc_data.rc.s1 = ((data[5] >> 4) & 0x0C) >> 2;
    rc_data.rc.s2 = ((data[5] >> 4) & 0x03);

    rc_data.mouse.x = ((int16_t)data[6]) | (static_cast<int16_t>(data[7]) << 8);
    rc_data.mouse.y = ((int16_t)data[8]) | (static_cast<int16_t>(data[9]) << 8);
    rc_data.mouse.z = ((int16_t)data[10]) | (static_cast<int16_t>(data[11]) << 8);

    rc_data.mouse.press_l = data[12];
    rc_data.mouse.press_r = data[13];

    rc_data.key.v = static_cast<int16_t>(data[14]);
}

void rc_ctrl_t::control_logic(dr16_ctrl_t *dr16_data) const
{
    dr16_data->rc.ch0 = static_cast<float>(rc_data.rc.ch0 - RC_CH_VALUE_OFFSET) / (RC_CH_VALUE_MAX - RC_CH_VALUE_MIN);
    dr16_data->rc.ch1 = static_cast<float>(rc_data.rc.ch1 - RC_CH_VALUE_OFFSET) / (RC_CH_VALUE_MAX - RC_CH_VALUE_MIN);
    dr16_data->rc.ch2 = static_cast<float>(rc_data.rc.ch2 - RC_CH_VALUE_OFFSET) / (RC_CH_VALUE_MAX - RC_CH_VALUE_MIN);
    dr16_data->rc.ch3 = static_cast<float>(rc_data.rc.ch3 - RC_CH_VALUE_OFFSET) / (RC_CH_VALUE_MAX - RC_CH_VALUE_MIN);

    dr16_data->rc.s1.sw_state = static_cast<sw_state_t>(rc_data.rc.s1);
    dr16_data->rc.s2.sw_state = static_cast<sw_state_t>(rc_data.rc.s2);

    dr16_data->mouse.x = static_cast<float>(rc_data.mouse.x) / 32768.0f;
    dr16_data->mouse.y = static_cast<float>(rc_data.mouse.y) / 32768.0f;
    dr16_data->mouse.z = static_cast<float>(rc_data.mouse.z) / 32768.0f;

    dr16_data->mouse.press_l.mouse_state = static_cast<mouse_state_t>(rc_data.mouse.press_l);
    dr16_data->mouse.press_r.mouse_state = static_cast<mouse_state_t>(rc_data.mouse.press_r);
}

void rc_ctrl_t::handle_irq()
{
    if (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart5);

        const uint16_t rx_len = UART5_MAX_RECV_LEN - __HAL_DMA_GET_COUNTER(huart5.hdmarx);
        HAL_UART_DMAStop(&huart5);

        if (rx_len >= sizeof(Rc_ctrl_t))
        {
            SCB_InvalidateDCache_by_Addr(reinterpret_cast<uint32_t*>(rx_buf[rx_buf_switch]), UART5_MAX_RECV_LEN);
            unpack_data(rx_buf[rx_buf_switch]);
        }

        rx_buf_switch = 1 - rx_buf_switch;
        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx_buf[rx_buf_switch], UART5_MAX_RECV_LEN);
        __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
    }
}