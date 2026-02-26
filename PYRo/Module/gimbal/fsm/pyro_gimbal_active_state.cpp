#include "pyro_gimbal_task.h"

namespace pyro
{
void gimbal_t::state_active_t::enter(gimbal_t *owner)
{
    owner->gimbal_ctx.motor.yaw_motor->enable();
    owner->gimbal_ctx.motor.pitch_motor->enable();
    owner->gimbal_ctx.motor.roll_motor->enable();
}

void gimbal_t::state_active_t::execute(
    gimbal_t *owner)
{
    owner->gimbal_ctx.data._target_yaw_angle += owner->gimbal_ctx.cmd->yaw_delta_angle;
    owner->gimbal_ctx.data._target_pitch_angle += owner->gimbal_ctx.cmd->pitch_delta_angle;
    owner->gimbal_ctx.data._target_roll_angle += owner->gimbal_ctx.cmd->roll_delta_angle;

    if (owner->gimbal_ctx.data._target_yaw_angle > yaw_max_value)
    {
        owner->gimbal_ctx.data._target_yaw_angle = yaw_max_value;
    }
    else if (owner->gimbal_ctx.data._target_yaw_angle < yaw_min_value)
    {
        owner->gimbal_ctx.data._target_yaw_angle = yaw_min_value;
    }

    if (owner->gimbal_ctx.data._target_pitch_angle > pitch_max_value)
    {
        owner->gimbal_ctx.data._target_pitch_angle = pitch_max_value;
    }
    else if (owner->gimbal_ctx.data._target_pitch_angle < pitch_min_value)
    {
        owner->gimbal_ctx.data._target_pitch_angle = pitch_min_value;
    }

    // if (owner->gimbal_ctx.data._target_roll_angle > roll_max_value)
    // {
    //     owner->gimbal_ctx.data._target_roll_angle = roll_max_value;
    // }
    // if (owner->gimbal_ctx.data._target_roll_angle < roll_min_value)
    // {
    //     owner->gimbal_ctx.data._target_roll_angle = roll_min_value;
    // }

    const float yaw_error = owner->gimbal_ctx.data._target_yaw_angle - owner->gimbal_ctx.data._current_yaw_angle;
    if (yaw_error > PI)
    {
        owner->gimbal_ctx.data._target_yaw_angle -= 2.0f * PI;
    }
    else if (yaw_error < -PI)
    {
        owner->gimbal_ctx.data._target_yaw_angle += 2.0f * PI;
    }

    gimbal_control(&owner->gimbal_ctx);
    send_motor_command(&owner->gimbal_ctx);
}

void gimbal_t::state_active_t::exit(
    gimbal_t *owner)
{

}

} // namespace pyro
