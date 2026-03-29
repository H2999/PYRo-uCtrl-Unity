#include "rc.h"
#include "stm32h7xx_hal_uart.h"
#include "task.h"
#include "usart.h"

rc_ctrl_t *rc_ctrl = nullptr;
dr16_ctrl_t dr16_data{};
Rc_ctrl_t data{};
struct frame
{
    float data[3];
    uint8_t tail[4];
}tx_data;

extern "C"
{
    void UART5_IRQHandler(void)
    {
        if (rc_ctrl != nullptr)
        {
            rc_ctrl->handle_irq();
        }

        HAL_UART_IRQHandler(&huart5);
    }

    void uav_rc_thread(void *argument)
    {
        tx_data.tail[0] = 0x00;
        tx_data.tail[1] = 0x00;
        tx_data.tail[2] = 0x80;
        tx_data.tail[3] = 0x7f;
        while (true)
        {
            rc_ctrl->control_logic(&dr16_data);
            vTaskDelay(1);
        }
    }

    void rc_demo(void *argument)
    {
        if (rc_ctrl == nullptr)
        {
            rc_ctrl = new rc_ctrl_t(&data);
        }
        rc_ctrl->init();
        xTaskCreate(uav_rc_thread, "uav_rc_thread", 256,
                   nullptr, configMAX_PRIORITIES - 3, nullptr);
        vTaskDelete(nullptr);
    }
}
