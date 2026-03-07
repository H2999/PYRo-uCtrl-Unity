#include "Gimbal/Uav/pyro_uav_gimbal.h"

namespace pyro
{
void uav_gimbal_t::state_active_t::enter(uav_gimbal_t *owner)
{
    owner->gimbal_ctx.motor.yaw_motor->enable();
    owner->gimbal_ctx.motor.pitch_motor->enable();
    owner->gimbal_ctx.motor.roll_motor->enable();

    owner->gimbal_ctx.data.correct_imu_ctx.yaw_offset = owner->gimbal_ctx.motor.yaw_motor->get_current_position();
    owner->gimbal_ctx.data.correct_imu_ctx.correct_flag = 1;
}

void uav_gimbal_t::state_active_t::execute(uav_gimbal_t *owner)
{
    //yaw目标值 归一化到-pi到pi之间
    owner->gimbal_ctx.data._target_yaw_angle += owner->gimbal_ctx.cmd->yaw_delta_angle;
    normalize_angle(owner->gimbal_ctx.data._target_yaw_angle);

    //pitch目标值 归一化到-pi到pi之间
    owner->gimbal_ctx.data._target_pitch_angle += owner->gimbal_ctx.cmd->pitch_delta_angle;
    normalize_angle(owner->gimbal_ctx.data._target_pitch_angle);

    //roll目标值
    owner->gimbal_ctx.data._target_roll_angle += owner->gimbal_ctx.cmd->roll_delta_angle;
    normalize_angle(owner->gimbal_ctx.data._target_roll_angle);

    if (owner->gimbal_ctx.data.pitch_motor_angle < 2.6f && owner->gimbal_ctx.data.pitch_motor_angle > 2.35f)
    {
        owner->gimbal_ctx.data.final_roll_angle = owner->gimbal_ctx.data._target_roll_angle;
    }
    else
    {
        if (owner->gimbal_ctx.data._current_yaw_speed > 0.01f)
        {
            owner->gimbal_ctx.data.final_roll_angle = owner->gimbal_ctx.data._target_roll_angle
                + owner->gimbal_ctx.data._current_yaw_speed * owner->gimbal_ctx.data._current_yaw_speed * yaw_to_roll;
        }
        else if (owner->gimbal_ctx.data._current_yaw_speed < -0.01f)
        {
            owner->gimbal_ctx.data.final_roll_angle = owner->gimbal_ctx.data._target_roll_angle
                - owner->gimbal_ctx.data._current_yaw_speed * owner->gimbal_ctx.data._current_yaw_speed * yaw_to_roll;
        }
        else
        {
            owner->gimbal_ctx.data.final_roll_angle = owner->gimbal_ctx.data._target_roll_angle;
        }
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

    //yaw轴过零点处理
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
