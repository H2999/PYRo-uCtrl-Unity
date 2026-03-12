#include "Uav_booster/Uav_booster.h"

namespace pyro
{
void uav_booster_t::fsm_active_t::on_enter(uav_booster_t *owner)
{
    change_state(&ready_state_fric_t);
}

void uav_booster_t::fsm_active_t::on_execute(uav_booster_t *owner)
{
    owner->booster_ctx.data_ctx.target_fric_speed[0] = owner->booster_ctx.cmd->target_fric1_speed;
    owner->booster_ctx.data_ctx.target_fric_speed[1] = owner->booster_ctx.cmd->target_fric2_speed;

    owner->fric_control();
    owner->send_fric_command();
}

void uav_booster_t::fsm_active_t::on_exit(uav_booster_t *owner)
{
}

}

