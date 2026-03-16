#include "pyro_module_base.h"
#include "pyro_rc_hub.h"
#include "pyro_uart_message.h"
#include "Gimbal/Uav/pyro_uav_gimbal.h"

using namespace pyro;
uav_gimbal_t *gimbal_ptr                       = nullptr;
uav_gimbal_cmd_t *gimbal_cmd_ptr               = nullptr;
dr16_drv_t::dr16_ctrl_t const *rc_ctrl_ptr = nullptr;
extern OperateBytes operate_bytes;

static constexpr float rc_sensitivity = 0.0025f;

extern "C"
{
void gimbal_rc2cmd(void const *rc_ctrl)
{
    read_scope_lock lock(
        rc_hub_t::get_instance(rc_hub_t::DR16)->get_lock());
    static auto *p_ctrl =
        static_cast<dr16_drv_t::dr16_ctrl_t const *>(rc_ctrl);

    //如果右侧拨码拨到上面 就进入无力模式
    if (dr16_drv_t::sw_state_t::SW_UP == p_ctrl->rc.s_r.state)
    {
        gimbal_cmd_ptr->mode = cmd_base_t::mode_t::PASSIVE;
        gimbal_cmd_ptr->yaw_delta_angle     = 0;
        gimbal_cmd_ptr->pitch_delta_angle   = 0;
        gimbal_cmd_ptr->roll_delta_angle    = 0;

        return;
    }
    gimbal_cmd_ptr->mode = cmd_base_t::mode_t::ACTIVE;


    if (dr16_drv_t::sw_state_t::SW_DOWN == p_ctrl->rc.s_r.state)
    {
        gimbal_cmd_ptr->auto_flag = true;

        gimbal_cmd_ptr->yaw_target_angle = operate_bytes.output_data.shoot_yaw;
        gimbal_cmd_ptr->pitch_target_angle = operate_bytes.output_data.shoot_pitch;
        gimbal_cmd_ptr->yaw_delta_angle   = 0;
        gimbal_cmd_ptr->pitch_delta_angle = 0;
        gimbal_cmd_ptr->roll_delta_angle  = 0;
    }
    if (dr16_drv_t::sw_state_t::SW_DOWN != p_ctrl->rc.s_r.state)
    {
        gimbal_cmd_ptr->auto_flag = false;

        gimbal_cmd_ptr->yaw_delta_angle   = - p_ctrl->rc.ch_rx * rc_sensitivity;
        gimbal_cmd_ptr->pitch_delta_angle = - p_ctrl->rc.ch_ry * rc_sensitivity;
        gimbal_cmd_ptr->roll_delta_angle  = - p_ctrl->rc.ch_lx * rc_sensitivity;
    }
    // else if (dr16_drv_t::sw_state_t::SW_MID == p_ctrl->rc.s_l.state)
    // {
    //     gimbal_cmd_ptr->auto_flag = false;
    // }
}

void uav_gimbal_main_thread(void *argument)
{
    while (true)
    {
        gimbal_rc2cmd(rc_ctrl_ptr);
        gimbal_ptr->set_command(*gimbal_cmd_ptr);
        vTaskDelay(1);
    }
}

status_t uav_gimbal_init(void *argument)
{
    gimbal_cmd_ptr     = new uav_gimbal_cmd_t();
    gimbal_ptr = uav_gimbal_t::instance();

    rc_ctrl_ptr        = static_cast<dr16_drv_t::dr16_ctrl_t const *>(
        rc_hub_t::get_instance(rc_hub_t::DR16)->read());

    gimbal_ptr->start();

    BaseType_t ret =
    xTaskCreate(uav_gimbal_main_thread, "uav_gimbal_main_thread", 512, nullptr,
                configMAX_PRIORITIES - 1, nullptr);

    CHECK_OS_RET(ret);
    return PYRO_OK;
}
}