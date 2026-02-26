// #include "pyro_uav_gimbal.h"
#include "Gimbal/Uav/pyro_uav_gimbal.h"

namespace pyro
{
void gimbal_t::state_active_t::enter(gimbal_t *owner)
{
    owner->gimbal_ctx.gimbal_cfg.motor_cfg->yaw_motor->enable();
    owner->gimbal_ctx.gimbal_cfg.motor_cfg->pitch_motor->enable();
    owner->gimbal_ctx.gimbal_cfg.motor_cfg->roll_motor->enable();
}

void gimbal_t::state_active_t::execute(
    gimbal_t *owner)
{
    owner->gimbal_ctx.data_ctx._target_yaw_angle += owner->gimbal_ctx.cmd->yaw_delta_angle;
    owner->gimbal_ctx.data_ctx._target_pitch_angle += owner->gimbal_ctx.cmd->pitch_delta_angle;
    owner->gimbal_ctx.data_ctx._target_roll_angle += owner->gimbal_ctx.cmd->roll_delta_angle;

    if (owner->gimbal_ctx.data_ctx._target_yaw_angle > yaw_max_value)
    {
        owner->gimbal_ctx.data_ctx._target_yaw_angle = yaw_max_value;
    }
    else if (owner->gimbal_ctx.data_ctx._target_yaw_angle < yaw_min_value)
    {
        owner->gimbal_ctx.data_ctx._target_yaw_angle = yaw_min_value;
    }

    if (owner->gimbal_ctx.data_ctx._target_pitch_angle > pitch_max_value)
    {
        owner->gimbal_ctx.data_ctx._target_pitch_angle = pitch_max_value;
    }
    else if (owner->gimbal_ctx.data_ctx._target_pitch_angle < pitch_min_value)
    {
        owner->gimbal_ctx.data_ctx._target_pitch_angle = pitch_min_value;
    }

    if (owner->gimbal_ctx.data_ctx._target_roll_angle > pitch_max_value)
    {
        owner->gimbal_ctx.data_ctx._target_roll_angle = pitch_max_value;
    }
    else if (owner->gimbal_ctx.data_ctx._target_roll_angle < pitch_min_value)
    {
        owner->gimbal_ctx.data_ctx._target_roll_angle = pitch_min_value;
    }

    const float yaw_error = owner->gimbal_ctx.data_ctx._target_yaw_angle - owner->gimbal_ctx.data_ctx._current_yaw_angle;
    if (yaw_error > PI)
    {
        owner->gimbal_ctx.data_ctx._target_yaw_angle -= 2.0f * PI;
    }
    else if (yaw_error < -PI)
    {
        owner->gimbal_ctx.data_ctx._target_yaw_angle += 2.0f * PI;
    }

    gimbal_control(&owner->gimbal_ctx);
    send_motor_command(&owner->gimbal_ctx);
}

void gimbal_t::state_active_t::exit(
    gimbal_t *owner)
{

}

} // namespace pyro
