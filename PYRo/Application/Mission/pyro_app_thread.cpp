#include "pyro_gimbal_task.h"
#include "pyro_module_base.h"
#include "pyro_mutex.h"
#include "pyro_rc_hub.h"

pyro::gimbal_t *gimbal_ptr                       = nullptr;
pyro::gimbal_cmd_t *gimbal_cmd_ptr               = nullptr;
pyro::dr16_drv_t::dr16_ctrl_t const *rc_ctrl_ptr = nullptr;

static constexpr float rc_sensitivity = 0.05f;
static constexpr float control_period = 0.001f;

extern "C"
{

    void gimbal_rc2cmd(void const *rc_ctrl)
    {
        pyro::scoped_mutex_t lock(gimbal_ptr->get_mutex());
        static auto *p_ctrl =
            static_cast<pyro::dr16_drv_t::dr16_ctrl_t const *>(rc_ctrl);

        //如果右侧拨码拨到上面 就进入无力模式
        if (pyro::dr16_drv_t::sw_state_t::SW_UP == p_ctrl->rc.s_r.state)
        {
            gimbal_cmd_ptr->mode = pyro::cmd_base_t::mode_t::ZERO_FORCE;
            gimbal_cmd_ptr->_target_yaw_angle     = 0;
            gimbal_cmd_ptr->_target_pitch_angle   = 0;
            gimbal_cmd_ptr->_target_roll_angle    = 0;
            return;
        }
        gimbal_cmd_ptr->mode = pyro::cmd_base_t::mode_t::ACTIVE;
        // 3. 映射摇杆数据
        // 注意：根据旧代码，vx = -Right_Y, vy = Right_X, wz = -Left_X, wy =
        // Left_Y * 0.002 新接口中，通道名称为 ch_rx, ch_ry, ch_lx, ch_ly
         gimbal_cmd_ptr->_target_yaw_angle    += p_ctrl->rc.ch_ry * control_period * rc_sensitivity;
         gimbal_cmd_ptr->_target_pitch_angle  += p_ctrl->rc.ch_rx * control_period * rc_sensitivity;
         gimbal_cmd_ptr->_target_roll_angle   += p_ctrl->rc.ch_lx * control_period * rc_sensitivity;
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
        gimbal_ptr = pyro::gimbal_t::instance();
        gimbal_cmd_ptr     = new pyro::gimbal_cmd_t();
        rc_ctrl_ptr        = static_cast<pyro::dr16_drv_t::dr16_ctrl_t const *>(
            pyro::rc_hub_t::get_instance(pyro::rc_hub_t::DR16)->read());
        xTaskCreate(start_app_thread, "start_app_thread", 512, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);
        vTaskDelete(nullptr);
    }
}