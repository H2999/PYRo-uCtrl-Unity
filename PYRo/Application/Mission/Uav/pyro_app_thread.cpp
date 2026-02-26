#include "pyro_module_base.h"
#include "pyro_rc_hub.h"
#include "Gimbal/Uav/pyro_uav_gimbal.h"

pyro::gimbal_t *gimbal_ptr                       = nullptr;
pyro::gimbal_cmd_t *gimbal_cmd_ptr               = nullptr;
pyro::dr16_drv_t::dr16_ctrl_t const *rc_ctrl_ptr = nullptr;

static constexpr float rc_sensitivity = 0.0035f;

extern "C"
{
    void gimbal_rc2cmd(void const *rc_ctrl)
    {
        pyro::read_scope_lock lock(
            pyro::rc_hub_t::get_instance(pyro::rc_hub_t::DR16)->get_lock());
        static auto *p_ctrl =
            static_cast<pyro::dr16_drv_t::dr16_ctrl_t const *>(rc_ctrl);

        //如果右侧拨码拨到上面 就进入无力模式
        if (pyro::dr16_drv_t::sw_state_t::SW_UP == p_ctrl->rc.s_r.state)
        {
            gimbal_cmd_ptr->mode = pyro::cmd_base_t::mode_t::PASSIVE;
            gimbal_cmd_ptr->yaw_delta_angle     = 0;
            gimbal_cmd_ptr->pitch_delta_angle   = 0;
            gimbal_cmd_ptr->roll_delta_angle    = 0;
            return;
        }
        gimbal_cmd_ptr->mode = pyro::cmd_base_t::mode_t::ACTIVE;

        gimbal_cmd_ptr->yaw_delta_angle   = p_ctrl->rc.ch_ry * rc_sensitivity;
        gimbal_cmd_ptr->pitch_delta_angle = p_ctrl->rc.ch_rx * rc_sensitivity;
        gimbal_cmd_ptr->roll_delta_angle  = p_ctrl->rc.ch_lx * rc_sensitivity;
    }

    void start_app_thread(void *argument)
    {
        gimbal_ptr->start();

        while (true)
        {
            gimbal_rc2cmd(rc_ctrl_ptr);
            gimbal_ptr->set_command(*gimbal_cmd_ptr);
            vTaskDelay(1);
        }
    }

    void pyro_app_init_thread(void *argument)
    {
        gimbal_cmd_ptr     = new pyro::gimbal_cmd_t();
        gimbal_ptr = pyro::gimbal_t::instance();
        rc_ctrl_ptr        = static_cast<pyro::dr16_drv_t::dr16_ctrl_t const *>(
            pyro::rc_hub_t::get_instance(pyro::rc_hub_t::DR16)->read());
        xTaskCreate(start_app_thread, "start_app_thread", 256, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);
        vTaskDelete(nullptr);
    }
}