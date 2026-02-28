#include "pyro_uav_shoot.h"

namespace pyro
{
void uav_shoot_t::fsm_active_state::on_enter(uav_shoot_t *owner)
{
    owner->shoot_ctx.motor->fric_motor[0]->enable();
    owner->shoot_ctx.motor->fric_motor[1]->enable();
    owner->shoot_ctx.motor->trigger_motor->enable();
}

void uav_shoot_t::fsm_active_state::on_execute(uav_shoot_t *ctx)
{
    if (ctx->shoot_ctx.cmd->is_ready)
    {
        this->change_state(&ready_state);
    }
    else if (ctx->shoot_ctx.cmd->is_fire)
    {
        this->change_state(&fire_state);
    }
}

void uav_shoot_t::fsm_active_state::on_exit(uav_shoot_t *owner)
{
}

}

