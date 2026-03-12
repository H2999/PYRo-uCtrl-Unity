#include "pyro_uart_comm.h"
#include "pyro_uart_drv.h"
#include <cstdint>

using namespace pyro;

struct boosterData_t {
    uint8_t  header;
    float    pitch;
    float    roll;
    uint16_t checksum;
};

boosterData_t receive_data;

uart_comm_t gimbal_comm(uart_drv_t::which_uart::uart7, 101, 1024);

void communication_setup()
{
    uint8_t frame_head = 0x55;                                              //待定
    gimbal_comm.register_msg_type<boosterData_t>(&frame_head, 1);
}

void gimbal_receiver_task(void *pvParameters)
{
    while (true)
    {
        if (gimbal_comm.read(receive_data, portMAX_DELAY))
        {
            // float current_pitch = my_data.pitch;
            // float current_roll  = my_data.roll;
        }
    }
}