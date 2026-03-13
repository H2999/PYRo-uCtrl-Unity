#include "pyro_core_dma_heap.h"
#include "pyro_uart_comm.h"
#include "pyro_uav_booster.h"
#include "pyro_crc.h"
#include "pyro_uart_message.h"

using namespace pyro;

uart_comm_t *uart_comm_ptr = nullptr;

__attribute__((section(".dma_heap"))) OperateBytes operate_bytes;
StateBytes state_bytes;
float read_time;
extern float avg_delay;

extern "C"
{
    void uav_pc_com_thread(void *argument)
    {
        vTaskDelay(500);
        while (true)
        {
            uart_comm_ptr->read(state_bytes, sizeof(StateBytes));
            operate_bytes.frame_header.sof = 0xA5;
            // operate_bytes.output_data.curr_yaw = direct_gimbal_t::instance()->get_ctx().data.current_yaw_rad;   // 应该显示为 00 00 80 3F
            // operate_bytes.output_data.curr_pitch = direct_gimbal_t::instance()->get_ctx().data.current_pitch_rad; // 应该显示为 00 00 00 40
            // operate_bytes.output_data.state = 0x00;
            // operate_bytes.output_data.autoaim = 0x01;
            // operate_bytes.output_data.enemy_color = 0x0;
            operate_bytes.output_data.curr_speed = 0.0f;
            // operate_bytes.output_data.shoot_delay = static_cast<uint16_t>(avg_delay);

            operate_bytes.output_data.curr_yaw = 0;   // 应该显示为 00 00 80 3F
            operate_bytes.output_data.curr_pitch = 0; // 应该显示为 00 00 00 40
            operate_bytes.output_data.state = 0x00;
            operate_bytes.output_data.autoaim = 0x0;
            operate_bytes.output_data.enemy_color = 0x0;
            operate_bytes.output_data.curr_speed = 0.0f;
            operate_bytes.output_data.shoot_delay = 0.0f;
            append_crc16_check_sum((uint8_t*)&operate_bytes,sizeof(OperateBytes) - 1);
            uart_drv_t::get_instance(uart_drv_t::which_uart::uart7)->write((uint8_t*)&operate_bytes, sizeof(OperateBytes));
            vTaskDelay(1);
        }
    }

    void uav_pc_com_init(void *argument)
    {
        uart_comm_ptr =
            new uart_comm_t(uart_drv_t::which_uart::uart7, 0x10, 256);
        uint8_t sof = 0xA5;
        uart_comm_ptr->register_msg_type(sizeof(StateBytes), &sof, 1);

        xTaskCreate(uav_pc_com_thread, "uav_pc_com_main_thread", 128,
                    nullptr, configMAX_PRIORITIES - 4, nullptr);
        vTaskDelete(nullptr);
    }
}
