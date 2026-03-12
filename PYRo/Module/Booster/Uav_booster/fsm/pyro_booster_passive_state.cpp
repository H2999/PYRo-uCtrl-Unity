#include "Booster/Uav_booster/Uav_booster.h"

namespace pyro
{
void uav_booster_t::passive_state_t::enter(uav_booster_t *owner)
{
    owner->booster_ctx.cfg.pid_cfg.fric_pid[0]->clear();
    owner->booster_ctx.cfg.pid_cfg.fric_pid[1]->clear();

    owner->booster_ctx.cfg.motor_cfg.fric_wheel[0]->disable();
    owner->booster_ctx.cfg.motor_cfg.fric_wheel[1]->disable();
}

void uav_booster_t::passive_state_t::execute(uav_booster_t *owner)
{
    owner->booster_ctx.data_ctx.fric_output_torque[0] = 0;
    owner->booster_ctx.data_ctx.fric_output_torque[1] = 0;

    owner->fric_control();
    owner->send_fric_command();
}

void uav_booster_t::passive_state_t::exit(uav_booster_t *owner)
{
}

}


