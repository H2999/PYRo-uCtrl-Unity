#include "pyro_module_base.h"
#include "pyro_rc_hub.h"
#include "pyro_uart_message.h"
#include "pyro_uav_booster.h"

using namespace pyro;
uav_booster_t *uav_booster_ptr           = nullptr;
uav_booster_cmd_t *uav_booster_cmd_ptr   = nullptr;
static dr16_drv_t::dr16_ctrl_t const *rc_ctrl_ptr = nullptr;
extern OperateBytes operate_bytes;

extern "C"
{
    void booster_rc2cmd(void const *rc_ctrl)
    {
        read_scope_lock lock(
            rc_hub_t::get_instance(rc_hub_t::DR16)->get_lock());
        static auto *p_ctrl =
            static_cast<dr16_drv_t::dr16_ctrl_t const *>(rc_ctrl);
        if (dr16_drv_t::sw_state_t::SW_UP == p_ctrl->rc.s_r.state)
        {
            uav_booster_cmd_ptr->mode = cmd_base_t::mode_t::PASSIVE;

            uav_booster_cmd_ptr->trigger_enable = false;
            uav_booster_cmd_ptr->single_mode = false;
            uav_booster_cmd_ptr->continue_mode = false;
            uav_booster_cmd_ptr->target_fric1_mps   = 0.0f;
            uav_booster_cmd_ptr->target_fric2_mps  = 0.0f;
            return;
        }

        if(dr16_drv_t::sw_state_t::SW_DOWN == p_ctrl->rc.s_r.state)
        {
            uav_booster_cmd_ptr->mode      = cmd_base_t::mode_t::ACTIVE;
        }

        if (dr16_drv_t::sw_ctrl_t::SW_MID_TO_UP == p_ctrl->rc.s_l.ctrl)
        {
            uav_booster_cmd_ptr->target_fric1_mps = -16.0f;
            uav_booster_cmd_ptr->target_fric2_mps = 16.0f;
        }
        if (dr16_drv_t::sw_ctrl_t::SW_UP_TO_MID == p_ctrl->rc.s_l.ctrl)
        {
            uav_booster_cmd_ptr->target_fric1_mps = 0.0f;
            uav_booster_cmd_ptr->target_fric2_mps = 0.0f;
        }

        // uav_booster_cmd_ptr->booster_auto_flag = operate_bytes.output_data.fire;
        if (dr16_drv_t::sw_ctrl_t::SW_MID_TO_DOWN == p_ctrl->rc.s_l.ctrl)
        {
            uav_booster_cmd_ptr->trigger_enable = true;
            uav_booster_cmd_ptr->continue_mode = true;
        }
        if (dr16_drv_t::sw_ctrl_t::SW_DOWN_TO_MID == p_ctrl->rc.s_l.ctrl)
        {
            uav_booster_cmd_ptr->trigger_enable = false;
            uav_booster_cmd_ptr->continue_mode = false;
        }

    }

    void uav_booster_thread(void *argument)
    {
        while (true)
        {
            booster_rc2cmd(rc_ctrl_ptr);
            uav_booster_ptr->set_command(*uav_booster_cmd_ptr);
            vTaskDelay(1);
        }
    }

    status_t uav_booster_init(void *argument)
    {
        uav_booster_ptr     = uav_booster_t::instance();
        uav_booster_cmd_ptr = new uav_booster_cmd_t();
        rc_ctrl_ptr = static_cast<dr16_drv_t::dr16_ctrl_t const *>(
            rc_hub_t::get_instance(rc_hub_t::DR16)->read());

        uav_booster_ptr->start();

        BaseType_t ret =
        xTaskCreate(uav_booster_thread, "uav_booster_main_thread", 256, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);
        CHECK_OS_RET(ret);
        return PYRO_OK;
    }
}