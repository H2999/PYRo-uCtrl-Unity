#include "pyro_core_dma_heap.h"
#include "pyro_uav_booster.h"
#include "pyro_crc.h"
#include "pyro_uav_gimbal.h"
#include "pc_info.h"
#include "usart.h"

using namespace pyro;

pc_communicate_t  *pc_communicate_ptr = nullptr;

OperateBytes operate_bytes;
__attribute__((section(".dma_heap"))) StateBytes state_bytes;
// extern uav_booster_t *uav_booster_ptr;
// extern uav_gimbal_t *gimbal_ptr;

extern "C"
{
    void UART7_IRQHandler(void)
    {
        if (pc_communicate_ptr != nullptr)
        {
            pc_communicate_ptr->handle_irq();
        }

        HAL_UART_IRQHandler(&huart7);
    }

    void uav_pc_thread(void *argument)
    {
        vTaskDelay(500);

        while (true)
        {
            state_bytes.frame_header.sof = 0xA5;
            // state_bytes.input_data.curr_yaw = gimbal_ptr->get_current_yaw_angle();
            // state_bytes.input_data.curr_pitch = - gimbal_ptr->get_current_pitch_angle();
            // state_bytes.input_data.curr_roll = gimbal_ptr->get_current_roll_angle();
            state_bytes.input_data.curr_speed = 0;

            state_bytes.input_data.shoot_delay = 0;
            state_bytes.input_data.state = 0;
            state_bytes.input_data.autoaim = 1;
            state_bytes.input_data.enemy_color = 0;
            state_bytes.frame_tailer.end = '\n';


            append_crc16_check_sum(reinterpret_cast<uint8_t *>(&state_bytes),sizeof(StateBytes) - 1);

            pc_communicate_ptr->tx_data_to_pc(&state_bytes);
            vTaskDelay(1);
        }
    }

    void uav_pc_com_init(void *argument)
    {
        if (pc_communicate_ptr == nullptr)
        {
            pc_communicate_ptr = new pc_communicate_t(&huart7);
        }
        pc_communicate_ptr->init();

        xTaskCreate(uav_pc_thread, "uav_pc_com_main_thread", 256,
                    nullptr, configMAX_PRIORITIES - 3, nullptr);
        vTaskDelete(nullptr);
    }
}
