#include "pyro_gimbal_task.h"

namespace pyro
{
void gimbal_t::fsm_active_t::state_motion_t::enter(gimbal_t *owner)
{
    // owner->_ctx.motor.track[0]->enable();
    // owner->_ctx.motor.track[1]->enable();
}

void gimbal_t::fsm_active_t::state_motion_t::execute(gimbal_t *owner)
{
    owner->gimbal_control();

    owner->send_motor_command();
}

void gimbal_t::fsm_active_t::state_motion_t::exit(gimbal_t *owner)
{
    // owner->_ctx.motor.track[0]->disable();
    // owner->_ctx.motor.track[1]->disable();
}
}