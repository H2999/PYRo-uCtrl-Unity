#include "pyro_core_dma_heap.h"
#include "pyro_uart_comm.h"
#include "pyro_uav_booster.h"
#include "pyro_crc.h"
#include "pyro_uart_message.h"
#include "pyro_uav_gimbal.h"

using namespace pyro;

uart_comm_t *uart_comm_ptr = nullptr;

OperateBytes operate_bytes;
__attribute__((section(".dma_heap"))) StateBytes state_bytes;
extern uav_booster_t *uav_booster_ptr;
extern uav_gimbal_t *gimbal_ptr;

extern "C"
{
    void uav_pc_com_thread(void *argument)
    {
        vTaskDelay(500);
        while (true)
        {
            uart_comm_ptr->read(operate_bytes, sizeof(OperateBytes));

            state_bytes.frame_header.sof = 0xA5;
            state_bytes.input_data.curr_yaw = gimbal_ptr->get_current_yaw_angle();
            state_bytes.input_data.curr_pitch = - gimbal_ptr->get_current_pitch_angle();
            state_bytes.input_data.curr_roll = gimbal_ptr->get_current_roll_angle();
            state_bytes.input_data.curr_speed = 0;

            state_bytes.input_data.shoot_delay = 0;
            state_bytes.input_data.state = 0;
            state_bytes.input_data.autoaim = 1;
            state_bytes.input_data.enemy_color = 0;
            state_bytes.frame_tailer.end = '\n';
            append_crc16_check_sum((uint8_t*)&state_bytes,sizeof(StateBytes) - 1);
            uart_drv_t::get_instance(uart_drv_t::which_uart::uart7)->write((uint8_t*)&state_bytes, sizeof(StateBytes));
            vTaskDelay(1);
        }
    }

    void uav_pc_com_init(void *argument)
    {
        uart_comm_ptr =
            new uart_comm_t(uart_drv_t::which_uart::uart7, 0x10, 256);
        uint8_t sof = 0xA5;
        uart_comm_ptr->register_msg_type(sizeof(OperateBytes), &sof, 1);

        xTaskCreate(uav_pc_com_thread, "uav_pc_com_main_thread", 256,
                    nullptr, configMAX_PRIORITIES - 4, nullptr);
        vTaskDelete(nullptr);
    }
}
