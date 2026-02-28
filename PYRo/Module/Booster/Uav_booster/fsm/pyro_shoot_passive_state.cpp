#include "Booster/Uav_booster/Uav_booster.h"

namespace pyro
{
void uav_booster_t::passive_state_t::enter(uav_booster_t *owner)
{
    // owner->booster_ctx.cfg.pid_cfg.fric_pid[0]->clear();
    // owner->booster_ctx.cfg.pid_cfg.fric_pid[1]->clear();
    owner->booster_ctx.cfg.pid_cfg.trigger_pos_pid->clear();
    owner->booster_ctx.cfg.pid_cfg.trigger_spd_pid->clear();

    trigger_stopped = false;

    // owner->booster_ctx.cfg.motor_cfg.fric_wheel[0]->disable();
    // owner->booster_ctx.cfg.motor_cfg.fric_wheel[1]->disable();
    // owner->booster_ctx.cfg.motor_cfg.trigger_wheel->disable();
}

void uav_booster_t::passive_state_t::execute(uav_booster_t *owner)
{
    owner->booster_ctx.data_ctx.fric_output_torque[0] = 0;
    owner->booster_ctx.data_ctx.fric_output_torque[1] = 0;
    owner->booster_ctx.data_ctx.trigger_output_torque = 0;

    // owner->booster_ctx.data_ctx.target_trigger_angle = owner->booster_ctx.data_ctx->current_trigger_angle;

    // send_command(&owner->booster_ctx);
}

// void uav_shoot_t::passive_state_t::exit(uav_shoot_t *owner)
// {
// }

}


