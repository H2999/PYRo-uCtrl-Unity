#include "pyro_gimbal_task.h"

namespace pyro
{

void gimbal_t::state_passive_t::enter(gimbal_t *owner)
{
    owner->gimbal_ctx->motor.yaw_motor->disable();
    owner->gimbal_ctx->motor.pitch_motor->disable();
    owner->gimbal_ctx->motor.roll_motor->disable();
}

void gimbal_t::state_passive_t::execute(gimbal_t *owner)
{
    owner->gimbal_ctx->motor.yaw_motor->send_torque(0);
    owner->gimbal_ctx->motor.pitch_motor->send_torque(0);
    owner->gimbal_ctx->motor.roll_motor->send_torque(0);
}

void gimbal_t::state_passive_t::exit(gimbal_t *owner)
{
}

} // namespace pyro