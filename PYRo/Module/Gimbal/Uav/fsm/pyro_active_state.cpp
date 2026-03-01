// #include "pyro_uav_gimbal.h"
#include "Gimbal/Uav/pyro_uav_gimbal.h"

namespace pyro
{
void uav_gimbal_t::state_active_t::enter(uav_gimbal_t *owner)
{
    owner->gimbal_ctx.motor.yaw_motor->enable();
    owner->gimbal_ctx.motor.pitch_motor->enable();
    owner->gimbal_ctx.motor.roll_motor->enable();

    owner->gimbal_ctx.data.correct_imu_ctx.yaw_offset = owner->gimbal_ctx.motor.yaw_motor->get_current_position();
}

void uav_gimbal_t::state_active_t::execute(uav_gimbal_t *owner)
{
    owner->gimbal_ctx.data._target_yaw_angle += owner->gimbal_ctx.cmd->yaw_delta_angle;

    //pitch目标值 后归一化到-pi到pi之间
    owner->gimbal_ctx.data._target_pitch_angle += owner->gimbal_ctx.cmd->pitch_delta_angle;
    if (owner->gimbal_ctx.data._target_pitch_angle > PI)
    {
        owner->gimbal_ctx.data._target_pitch_angle -= 2.0f * PI;
    }
    else if (owner->gimbal_ctx.data._target_pitch_angle < -PI)
    {
        owner->gimbal_ctx.data._target_pitch_angle += 2.0f * PI;
    }

    //roll目标值 后归一化到-pi到pi之间
    owner->gimbal_ctx.data._target_roll_angle += owner->gimbal_ctx.cmd->roll_delta_angle;
    if (owner->gimbal_ctx.data._target_roll_angle > PI)
    {
        owner->gimbal_ctx.data._target_roll_angle -= 2.0f * PI;
    }
    else if (owner->gimbal_ctx.data._target_roll_angle < -PI)
    {
        owner->gimbal_ctx.data._target_roll_angle += 2.0f * PI;
    }

    //yaw轴限位
    if (owner->gimbal_ctx.data._target_yaw_angle < yaw_max_value && owner->gimbal_ctx.data._target_yaw_angle > 0)
    {
        owner->gimbal_ctx.data._target_yaw_angle = yaw_max_value;
    }
    else if (owner->gimbal_ctx.data._target_yaw_angle > yaw_min_value && owner->gimbal_ctx.data._target_yaw_angle < 0)
    {
        owner->gimbal_ctx.data._target_yaw_angle = yaw_min_value;
    }

    //pitch轴限位
    if (owner->gimbal_ctx.data._target_pitch_angle > pitch_max_value)
    {
        owner->gimbal_ctx.data._target_pitch_angle = pitch_max_value;
    }
    else if (owner->gimbal_ctx.data._target_pitch_angle < pitch_min_value)
    {
        owner->gimbal_ctx.data._target_pitch_angle = pitch_min_value;
    }

    //roll轴限位
    if (owner->gimbal_ctx.data._target_roll_angle > roll_max_value)
    {
        owner->gimbal_ctx.data._target_roll_angle = roll_max_value;
    }
    else if (owner->gimbal_ctx.data._target_roll_angle < roll_min_value)
    {
        owner->gimbal_ctx.data._target_roll_angle = roll_min_value;
    }

    //yaw过零点处理
    const float yaw_error = owner->gimbal_ctx.data._target_yaw_angle - owner->gimbal_ctx.data.correct_imu_ctx.correct_yaw_angle;
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

void uav_gimbal_t::state_active_t::exit(uav_gimbal_t *owner)
{

}

} // namespace pyro
