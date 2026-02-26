#include "pyro_uav_shoot.h"

namespace pyro
{
void uav_shoot_t::passive_state_t::enter(uav_shoot_t *owner)
{
    owner->shoot_ctx.pid->fric_speed_pid[0]->clear();
    owner->shoot_ctx.pid->fric_speed_pid[1]->clear();
    owner->shoot_ctx.pid->trigger_position_pid->clear();
    owner->shoot_ctx.pid->trigger_speed_pid->clear();

    owner->shoot_ctx.motor->fric_motor[0]->disable();
    owner->shoot_ctx.motor->fric_motor[1]->disable();
    owner->shoot_ctx.motor->trigger_motor->disable();
}

void uav_shoot_t::passive_state_t::execute(uav_shoot_t *owner)
{
    owner->shoot_ctx.data->output_fric_torque[0] = 0;
    owner->shoot_ctx.data->output_fric_torque[1] = 0;
    owner->shoot_ctx.data->output_trigger_torque = 0;

    owner->shoot_ctx.data->target_trigger_angle = owner->shoot_ctx.data->current_trigger_angle;

    send_cammand(&owner->shoot_ctx);
}

void uav_shoot_t::passive_state_t::exit(uav_shoot_t *owner)
{
}

}


