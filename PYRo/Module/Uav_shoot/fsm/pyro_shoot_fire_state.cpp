#include "pyro_uav_shoot.h"

namespace pyro
{
void uav_shoot_t::fsm_active_state::fire_state_t::enter(uav_shoot_t *owner)
{

}

void uav_shoot_t::fsm_active_state::fire_state_t::execute(uav_shoot_t *owner)
{
    if(owner->shoot_ctx.is_block->block_flag == true)
    {
        owner->shoot_ctx.data->target_trigger_speed = block_time;
        owner->shoot_ctx.is_block->adjust_time ++;
        if (owner->shoot_ctx.is_block->adjust_time == adjust_time)
        {
            owner->shoot_ctx.is_block->block_flag = false;
            owner->shoot_ctx.is_block->block_time = 0;
            owner->shoot_ctx.is_block->adjust_time = 0;
        }
    }

    else
    {
        owner->shoot_ctx.data->target_fric_speed[0] = 0.5;
        owner->shoot_ctx.data->target_fric_speed[1] = 0.5;
        owner->shoot_ctx.data->target_trigger_speed = 0.5;

        shoot_control(&owner->shoot_ctx);
        send_cammand(&owner->shoot_ctx);
    }
}

void uav_shoot_t::fsm_active_state::fire_state_t::exit(uav_shoot_t *owner)
{

}

}