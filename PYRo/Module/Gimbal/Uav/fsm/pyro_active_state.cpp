#include "Gimbal/Uav/pyro_uav_gimbal.h"

#include <functional>

namespace pyro
{
void uav_gimbal_t::state_active_t::enter(uav_gimbal_t *owner)
{
    owner->gimbal_ctx.motor.yaw_motor->enable();
    owner->gimbal_ctx.motor.pitch_motor->enable();
    owner->gimbal_ctx.motor.roll_motor->enable();

    owner->gimbal_ctx.data.correct_imu_ctx.correct_flag = 1;

    if (owner->gimbal_ctx.data.correct_imu_ctx.correct_flag == 1)
    {
        owner->gimbal_ctx.data.correct_imu_ctx.yaw_offset = owner->gimbal_ctx.motor.yaw_motor->get_current_position();
    }
}

void uav_gimbal_t::state_active_t::execute(uav_gimbal_t *owner)
{
    //yaw目标值 归一化到-pi到pi之间
    owner->gimbal_ctx.data._target_yaw_angle += owner->gimbal_ctx.cmd->yaw_delta_angle;
    normalize_angle(owner->gimbal_ctx.data._target_yaw_angle);

    if (owner->gimbal_ctx.data._target_yaw_angle < yaw_max_value && owner->gimbal_ctx.data._target_yaw_angle > 0)
    {
        owner->gimbal_ctx.data._target_yaw_angle = yaw_max_value;
    }
    else if (owner->gimbal_ctx.data._target_yaw_angle > yaw_min_value && owner->gimbal_ctx.data._target_yaw_angle < 0)
    {
        owner->gimbal_ctx.data._target_yaw_angle = yaw_min_value;
    }

    //pitch目标值 归一化到-pi到pi之间
    owner->gimbal_ctx.data._target_pitch_angle += owner->gimbal_ctx.cmd->pitch_delta_angle;
    normalize_angle(owner->gimbal_ctx.data._target_pitch_angle);

    if (owner->gimbal_ctx.data._target_pitch_angle > pitch_max_value)
    {
        owner->gimbal_ctx.data._target_pitch_angle = pitch_max_value;
    }
    else if (owner->gimbal_ctx.data._target_pitch_angle < pitch_min_value)
    {
        owner->gimbal_ctx.data._target_pitch_angle = pitch_min_value;
    }

    // roll目标值 归一化到-pi到pi之间
     owner->gimbal_ctx.data._target_roll_angle += owner->gimbal_ctx.cmd->roll_delta_angle;
     normalize_angle(owner->gimbal_ctx.data._target_roll_angle);
//3.04 2.78
     static float yaw_to_roll = 0.0f;
     if (owner->gimbal_ctx.data.pitch_motor_angle < 2.78f && owner->gimbal_ctx.data.pitch_motor_angle > 2.3f)
     {
         owner->gimbal_ctx.data.final_roll_angle = owner->gimbal_ctx.data._target_roll_angle;
     }
     else
     {
         if (owner->gimbal_ctx.data.pitch_motor_angle < 3.04f && owner->gimbal_ctx.data.pitch_motor_angle > 2.78f)
         {
             yaw_to_roll = 0.04;
         }
         else
         {
             yaw_to_roll = 0.065f;
         }
         if (owner->gimbal_ctx.data._current_yaw_speed > 0.4f)
         {
             owner->gimbal_ctx.data.final_roll_angle = owner->gimbal_ctx.data._target_roll_angle
                 + owner->gimbal_ctx.data._current_yaw_speed * owner->gimbal_ctx.data._current_yaw_speed * yaw_to_roll;
         }
         else if (owner->gimbal_ctx.data._current_yaw_speed < -0.4f)
         {
             owner->gimbal_ctx.data.final_roll_angle = owner->gimbal_ctx.data._target_roll_angle
                 - owner->gimbal_ctx.data._current_yaw_speed * owner->gimbal_ctx.data._current_yaw_speed * yaw_to_roll;
         }
         else
         {
             owner->gimbal_ctx.data.final_roll_angle = owner->gimbal_ctx.data._target_roll_angle;
         }
     }

     if (owner->gimbal_ctx.data._target_roll_angle > roll_max_value)
     {
         owner->gimbal_ctx.data._target_roll_angle = roll_max_value;
     }
     else if (owner->gimbal_ctx.data._target_roll_angle < roll_min_value)
     {
         owner->gimbal_ctx.data._target_roll_angle = roll_min_value;
     }

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

// void uav_gimbal_t::state_active_t::execute(uav_gimbal_t *owner)
// {
//     //yaw目标值 归一化到-pi到pi之间
//     owner->gimbal_ctx.data.yaw_angle_increase = owner->gimbal_ctx.data._target_yaw_angle - owner->gimbal_ctx.data.yaw_motor_angle;
//
//     //yaw轴限位
//     // if ((owner->gimbal_ctx.data._current_yaw_angle + owner->gimbal_ctx.data.yaw_angle_increase) < yaw_max_value
//     //     && (owner->gimbal_ctx.data._current_yaw_angle + owner->gimbal_ctx.data.yaw_angle_increase) > 0)
//     // {
//     //     owner->gimbal_ctx.data._target_yaw_angle = yaw_max_value;
//     // }
//     // else if ((owner->gimbal_ctx.data._current_yaw_angle + owner->gimbal_ctx.data.yaw_angle_increase) > yaw_min_value
//     //     && (owner->gimbal_ctx.data._current_yaw_angle + owner->gimbal_ctx.data.yaw_angle_increase) < 0)
//     // {
//     //     owner->gimbal_ctx.data._target_yaw_angle = yaw_min_value;
//     // }
//     // else
//     // {
//     //     owner->gimbal_ctx.data._target_yaw_angle = owner->gimbal_ctx.data._current_yaw_angle + owner->gimbal_ctx.data.yaw_angle_increase;
//     // }
//     owner->gimbal_ctx.data._target_yaw_angle += owner->gimbal_ctx.cmd->yaw_delta_angle;
//     normalize_angle(owner->gimbal_ctx.data._target_yaw_angle);
//
//
//
//     //pitch目标值 归一化到-pi到pi之间
//     // owner->gimbal_ctx.data.pitch_angle_error = owner->gimbal_ctx.data._target_pitch_angle - owner->gimbal_ctx.data.pitch_motor_angle;
//     // normalize_angle(owner->gimbal_ctx.data.pitch_angle_error);
//     //
//     // if (owner->gimbal_ctx.data._current_pitch_angle + owner->gimbal_ctx.data.pitch_angle_error + owner->gimbal_ctx.cmd->pitch_delta_angle > pitch_max_value)
//     // {
//     //     if (owner->gimbal_ctx.cmd->pitch_delta_angle > 0.0f)
//     //     {
//     //         owner->gimbal_ctx.cmd->pitch_delta_angle = pitch_max_value -
//     //             owner->gimbal_ctx.data._current_pitch_angle - owner->gimbal_ctx.data.pitch_angle_error;
//     //     }
//     // }
//     // else if (owner->gimbal_ctx.data._current_pitch_angle + owner->gimbal_ctx.data.pitch_angle_error + owner->gimbal_ctx.data.pitch_angle_increase < pitch_min_value)
//     // {
//     //     if (owner->gimbal_ctx.cmd->pitch_delta_angle < 0.0f)
//     //     {
//     //         owner->gimbal_ctx.cmd->pitch_delta_angle = pitch_min_value -
//     //             owner->gimbal_ctx.data._current_pitch_angle - owner->gimbal_ctx.data.pitch_angle_error;
//     //     }
//     // }
//     // owner->gimbal_ctx.data._target_pitch_angle += owner->gimbal_ctx.cmd->pitch_delta_angle;
//     // normalize_angle(owner->gimbal_ctx.data._target_pitch_angle);
//
//
//
//     if (owner->gimbal_ctx.data.roll_motor_angle + owner->gimbal_ctx.cmd->roll_delta_angle > roll_max_value)
//      {
//          owner->gimbal_ctx.cmd->roll_delta_angle = roll_max_value - owner->gimbal_ctx.data.roll_motor_angle;
//      }
//     else if (owner->gimbal_ctx.data.roll_motor_angle + owner->gimbal_ctx.cmd->roll_delta_angle < roll_min_value)
//      {
//          owner->gimbal_ctx.cmd->roll_delta_angle = roll_min_value - owner->gimbal_ctx.data.roll_motor_angle;
//      }
//     owner->gimbal_ctx.data._target_roll_angle += owner->gimbal_ctx.cmd->roll_delta_angle;
//     normalize_angle(owner->gimbal_ctx.data._target_roll_angle);
//
//     gimbal_control(&owner->gimbal_ctx);
//     send_motor_command(&owner->gimbal_ctx);
// }
//
// void uav_gimbal_t::state_active_t::exit(uav_gimbal_t *owner)
// {
//
// }

} // namespace pyro
