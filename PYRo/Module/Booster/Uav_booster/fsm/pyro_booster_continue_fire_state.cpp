#include "pyro_uav_booster.h"
using namespace pyro;

void uav_booster_t::fsm_active_t::shoot_continue_bullet_t::enter(uav_booster_t *owner)
{

}

void uav_booster_t::fsm_active_t::shoot_continue_bullet_t::execute(uav_booster_t *owner)
{
    // if (owner->booster_ctx.cmd->trigger_enable)
    // {
        owner->booster_ctx.data_ctx.target_trigger_radps = 1.0f; //待定
    // }

    owner->trigger_speed_control();
    owner->send_trigger_command();
}

void uav_booster_t::fsm_active_t::shoot_continue_bullet_t::exit(uav_booster_t *owner)
{

}