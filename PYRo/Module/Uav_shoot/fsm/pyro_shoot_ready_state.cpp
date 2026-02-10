//
// Created by huifeng on 2026/2/3.
//

#include "pyro_uav_shoot.h"

namespace  pyro
{
void uav_shoot_t::fsm_active_state::ready_state_t::enter(
    uav_shoot_t *owner)
{

}

void uav_shoot_t::fsm_active_state::ready_state_t::execute(
    uav_shoot_t *owner)
{
    owner->shoot_ctx.data->target_fric_speed[0] = 0.5;
    owner->shoot_ctx.data->target_fric_speed[1] = 0.5;

    shoot_control(&owner->shoot_ctx);
    send_cammand(&owner->shoot_ctx);
}

void uav_shoot_t::fsm_active_state::ready_state_t::exit(uav_shoot_t *owner)
{

}
}