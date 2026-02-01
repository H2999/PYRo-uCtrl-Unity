#include "pyro_gimbal_task.h"

namespace pyro
{
class gimbal_t;

void gimbal_t::fsm_active_t::on_enter(gimbal_t *owner)
{
    owner->gimbal_ctx->motor.yaw_motor->enable();
    owner->gimbal_ctx->motor.pitch_motor->enable();
    owner->gimbal_ctx->motor.roll_motor->enable();
}

void gimbal_t::fsm_active_t::on_execute(
    gimbal_t *owner)
{
    if (owner->gimbal_ctx->cmd->mode == cmd_base_t::mode_t::ACTIVE)
    {
        this->change_state(&_state_motion);
    }
}

void gimbal_t::fsm_active_t::on_exit(
    gimbal_t *owner)
{

}

} // namespace pyro
