#include "pyro_module_base.h"
#include "pyro_rc_hub.h"
#include "Quad_tango/pyro_quad_booster.h"
#include "Uav_booster/Uav_booster.h"

static pyro::uav_booster_t *uav_booster_ptr           = nullptr;
static pyro::uav_booster_cmd_t *quad_booster_cmd_ptr   = nullptr;
static pyro::dr16_drv_t::dr16_ctrl_t const *rc_ctrl_ptr = nullptr;

extern "C"
{
    void booster_rc2cmd(void const *rc_ctrl)
    {
        pyro::read_scope_lock lock(
            pyro::rc_hub_t::get_instance(pyro::rc_hub_t::DR16)->get_lock());
        static auto *p_ctrl =
            static_cast<pyro::dr16_drv_t::dr16_ctrl_t const *>(rc_ctrl);
        if (pyro::dr16_drv_t::sw_state_t::SW_UP == p_ctrl->rc.s_r.state)
        {
            quad_booster_cmd_ptr->mode = pyro::cmd_base_t::mode_t::PASSIVE;
            // quad_booster_cmd_ptr->fric1_speed   = 0.0f;
            // quad_booster_cmd_ptr->fric2_speed  = 0.0f;
            return;
        }
        quad_booster_cmd_ptr->mode      = pyro::cmd_base_t::mode_t::ACTIVE;
        // quad_booster_cmd_ptr->fric1_speed = 14.0f; // 可调节
        // quad_booster_cmd_ptr->fric2_mps = 14.0f;
        // 摩擦轮控制
        static float sl_using_time      = 0;
        if (pyro::dr16_drv_t::sw_ctrl_t::SW_UP_TO_MID == p_ctrl->rc.s_l.ctrl &&
            p_ctrl->rc.s_l.change_time != sl_using_time)
        {
            sl_using_time                 = p_ctrl->rc.s_l.change_time;
            quad_booster_cmd_ptr->fric_on = !quad_booster_cmd_ptr->fric_on;
        }
        if (pyro::dr16_drv_t::sw_ctrl_t::SW_DOWN_TO_MID ==
                p_ctrl->rc.s_l.ctrl &&
            p_ctrl->rc.s_l.change_time != sl_using_time)
        {
            sl_using_time                     = p_ctrl->rc.s_l.change_time;
            // quad_booster_cmd_ptr->fire_enable = true;
        }
        else
        {
            // quad_booster_cmd_ptr->fire_enable = false;
        }
        // 开火控制 (单发）
    }


    void hero_booster_thread(void *argument)
    {
        while (true)
        {
            booster_rc2cmd(rc_ctrl_ptr);
            uav_booster_ptr->set_command(*quad_booster_cmd_ptr);
            vTaskDelay(1);
        }
    }

    void hero_booster_init(void *argument)
    {
        // uav_booster_ptr     = pyro::quad_booster_t::instance();
        // quad_booster_cmd_ptr = new pyro::quad_booster_cmd_t();
        rc_ctrl_ptr = static_cast<pyro::dr16_drv_t::dr16_ctrl_t const *>(
            pyro::rc_hub_t::get_instance(pyro::rc_hub_t::DR16)->read());
        uav_booster_ptr->start();
        xTaskCreate(hero_booster_thread, "start_app_thread", 128, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);
        vTaskDelete(nullptr);
    }
}