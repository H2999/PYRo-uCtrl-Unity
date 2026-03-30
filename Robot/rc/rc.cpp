#include "rc.h"

#include <cstring>
#include "usart.h"

using namespace pyro;

rc_ctrl_t* rc_ctrl_t::instance = nullptr;
rc_ctrl_t::rc_ctrl_t(Rc_ctrl_t *data)
{
    instance = this;
    rx_buf[0] = static_cast<uint8_t *>(pvPortDmaMalloc(128));
    rx_buf[1] = static_cast<uint8_t *>(pvPortDmaMalloc(128));
    rc_data = data;
}

rc_ctrl_t::~rc_ctrl_t()
{
    vPortDmaFree(rx_buf[0]);
    vPortDmaFree(rx_buf[1]);
}

void rc_ctrl_t::unpack_data(uint8_t *data)
{
    rc_data->rc.ch0 = ((static_cast<int16_t>(data[0])) | (static_cast<int16_t>(data[1]) << 8)) & 0x07FF;
    rc_data->rc.ch1 = (static_cast<int16_t>(data[1]) >> 3 | (static_cast<int16_t>(data[2]) << 5)) & 0x07FF;
    rc_data->rc.ch2 = (static_cast<int16_t>(data[2]) >> 6 | (static_cast<int16_t>(data[3]) << 2) | (static_cast<int16_t>(data[4]) << 10)) & 0x07FF;
    rc_data->rc.ch3 = (static_cast<int16_t>(data[4]) >> 1 | (static_cast<int16_t>(data[5]) << 7)) & 0x07FF;

    rc_data->rc.s1 = ((data[5] >> 4) & 0x0C) >> 2;
    rc_data->rc.s2 = ((data[5] >> 4) & 0x03);

    rc_data->mouse.x = ((int16_t)data[6]) | (static_cast<int16_t>(data[7]) << 8);
    rc_data->mouse.y = ((int16_t)data[8]) | (static_cast<int16_t>(data[9]) << 8);
    rc_data->mouse.z = ((int16_t)data[10]) | (static_cast<int16_t>(data[11]) << 8);

    rc_data->mouse.press_l = data[12];
    rc_data->mouse.press_r = data[13];

    rc_data->key.v = static_cast<int16_t>(data[14]);
}

void rc_ctrl_t::control_logic(dr16_ctrl_t *dr16_data) const
{
    dr16_data->rc.ch0 = static_cast<float>(rc_data->rc.ch0 - RC_CH_VALUE_OFFSET) / (RC_CH_VALUE_MAX - RC_CH_VALUE_MIN);
    dr16_data->rc.ch1 = static_cast<float>(rc_data->rc.ch1 - RC_CH_VALUE_OFFSET) / (RC_CH_VALUE_MAX - RC_CH_VALUE_MIN);
    dr16_data->rc.ch2 = static_cast<float>(rc_data->rc.ch2 - RC_CH_VALUE_OFFSET) / (RC_CH_VALUE_MAX - RC_CH_VALUE_MIN);
    dr16_data->rc.ch3 = static_cast<float>(rc_data->rc.ch3 - RC_CH_VALUE_OFFSET) / (RC_CH_VALUE_MAX - RC_CH_VALUE_MIN);

    dr16_data->rc.s1.sw_state = static_cast<sw_state_t>(rc_data->rc.s1);
    dr16_data->rc.s2.sw_state = static_cast<sw_state_t>(rc_data->rc.s2);

    dr16_data->mouse.x = static_cast<float>(rc_data->mouse.x) / 32768.0f;
    dr16_data->mouse.y = static_cast<float>(rc_data->mouse.y) / 32768.0f;
    dr16_data->mouse.z = static_cast<float>(rc_data->mouse.z) / 32768.0f;

    dr16_data->mouse.press_l.mouse_state = static_cast<mouse_state_t>(rc_data->mouse.press_l);
    dr16_data->mouse.press_r.mouse_state = static_cast<mouse_state_t>(rc_data->mouse.press_r);
}

void rc_ctrl_t::init()
{
    // 1. 强行解除 HAL 库的软件锁 (这是解决 HAL_LOCKED 的唯一办法)
    huart5.Lock = HAL_UNLOCKED;
    if (huart5.hdmarx) {
        huart5.hdmarx->Lock = HAL_UNLOCKED;
        huart5.hdmarx->State = HAL_DMA_STATE_READY;
    }

    // 2. 先彻底停止 DMA 搬运工
    HAL_DMA_Abort(huart5.hdmarx);

    // 3. 物理配置串口 (核心：解决 NDTR=25 不动的问题)
    UART5->CR1 &= ~USART_CR1_UE;
    UART5->CR2 |= USART_CR2_RXINV; // S.Bus 必须电平反转！
    UART5->CR1 |= USART_CR1_UE;

    // 4. 清理串口所有的陈年垃圾标志位
    UART5->ICR = 0xFFFFFFFF;
    volatile uint32_t dummy = UART5->RDR;
    (void)dummy;

    // 5. 重新启动接收
    huart5.RxState = HAL_UART_STATE_READY;
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx_buf[0], 25) != HAL_OK) {
        // 如果这里还返回错误，说明硬件配置还是没进去
        // 可以尝试直接操作寄存器开启 DMAR:
        SET_BIT(UART5->CR3, USART_CR3_DMAR);
    }

    // 6. 禁用半传输中断，降低 CPU 负担
    __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
}

void rc_ctrl_t::wrtie(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    HAL_UART_Transmit(huart, pData, Size, Timeout);
}

// void rc_ctrl_t::init()
// {
//     // HAL_UART_RegisterRxEventCallback(&huart5, uart_rx_wrapper);
//     HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx_buf[0], 18);
//     __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
// }

// void rc_ctrl_t::uart_rx_wrapper(UART_HandleTypeDef *huart,const uint16_t Size)
// {
//     if (instance != nullptr)
//     {
//         instance->handle_rx_event(Size);
//     }
// }

// void rc_ctrl_t::handle_rx_event(uint16_t size)
// {
//     __HAL_UART_CLEAR_FLAG(&huart5, UART_CLEAR_PEF | UART_CLEAR_FEF |
//                                           UART_CLEAR_NEF | UART_CLEAR_OREF |
//                                           UART_CLEAR_RTOF);
//     SCB_InvalidateDCache_by_Addr((uint32_t*)rx_buf[rx_buf_switch], 18); // 建议 32 字节对齐
//
//     // if (size >= 18) {
//     //     // unpack_data()
//     // }
//
//     rx_buf_switch = 1 - rx_buf_switch;
//     HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx_buf[rx_buf_switch], 18);
//
// }

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance != UART5 || rc_ctrl_t::instance == nullptr) return;

    // --- 核心修复：如果 ISR 还是错的，直接重置硬件而非继续执行 ---
    if (huart->Instance->ISR & USART_ISR_ORE) {
        huart->Instance->ICR = USART_ICR_ORECF; // 清除溢出
        volatile uint32_t d = huart->Instance->RDR;
        (void)d;
        goto restart;
    }

    SCB_InvalidateDCache_by_Addr((uint32_t*)rc_ctrl_t::instance->rx_buf[rc_ctrl_t::instance->rx_buf_switch], 32);
    rc_ctrl_t::instance->unpack_data(rc_ctrl_t::instance->rx_buf[rc_ctrl_t::instance->rx_buf_switch]);

    restart:
        rc_ctrl_t::instance->rx_buf_switch ^= 1;
    // 强制关闭可能被 HAL 库误开启的 TXFEIE
    CLEAR_BIT(huart->Instance->CR1, USART_CR1_TXFEIE);

    huart->RxState = HAL_UART_STATE_READY;
    HAL_UARTEx_ReceiveToIdle_DMA(huart, rc_ctrl_t::instance->rx_buf[rc_ctrl_t::instance->rx_buf_switch], 18);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART5)
    {
        // 发现错误（如 ORE），清理现场并原地复活
        huart->Instance->ICR = 0xFFFFFFFF;
        volatile uint32_t d = huart->Instance->RDR;
        (void)d;

        // 重新启动接收
        rc_ctrl_t::instance->init();
    }
}