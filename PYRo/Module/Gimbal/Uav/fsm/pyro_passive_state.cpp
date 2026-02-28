#include "Gimbal/Uav/pyro_uav_gimbal.h"

namespace pyro
{

void uav_gimbal_t::state_passive_t::enter(uav_gimbal_t *owner)
{
    owner->gimbal_ctx.pid.yaw_position_pid->clear();
    owner->gimbal_ctx.pid.pitch_position_pid->clear();
    owner->gimbal_ctx.pid.roll_position_pid->clear();

    owner->gimbal_ctx.pid.yaw_speed_pid->clear();
    owner->gimbal_ctx.pid.pitch_speed_pid->clear();
    owner->gimbal_ctx.pid.roll_speed_pid->clear();

    owner->gimbal_ctx.motor.yaw_motor->disable();
    owner->gimbal_ctx.motor.pitch_motor->disable();
    owner->gimbal_ctx.motor.roll_motor->disable();

    //读取刚上电时电机的角度作为偏移量 用户后续加上IMU读取出来的角度得到绝对编码
    owner->gimbal_ctx.data.correct_imu_ctx.yaw_offset = owner->gimbal_ctx.motor.yaw_motor->get_current_position();
    owner->gimbal_ctx.data._target_yaw_angle = owner->gimbal_ctx.data.correct_imu_ctx.correct_yaw_angle;

    owner->gimbal_ctx.data.correct_imu_ctx.pitch_offset = owner->gimbal_ctx.motor.pitch_motor->get_current_position();
    owner->gimbal_ctx.data.correct_imu_ctx.roll_offset = owner->gimbal_ctx.motor.roll_motor->get_current_position();
}

void uav_gimbal_t::state_passive_t::execute(uav_gimbal_t *owner)
{
    owner->gimbal_ctx.data._output_yaw_torque = 0;
    owner->gimbal_ctx.data._output_pitch_torque = 0;
    owner->gimbal_ctx.data._output_roll_torque = 0;

    //下力时停在当前位置 防止下次上力的时候抽动回下力的位置
    owner->gimbal_ctx.data._target_yaw_angle = owner->gimbal_ctx.data.correct_imu_ctx.correct_yaw_angle;
    owner->gimbal_ctx.data._target_pitch_angle = owner->gimbal_ctx.data.correct_imu_ctx.correct_pitch_angle;
    owner->gimbal_ctx.data._target_roll_angle = owner->gimbal_ctx.data.correct_imu_ctx.correct_roll_angle;

    owner->gimbal_ctx.data._target_yaw_speed = 0;
    owner->gimbal_ctx.data._target_pitch_speed = 0;
    owner->gimbal_ctx.data._target_roll_speed = 0;

    send_motor_command(&owner->gimbal_ctx);
}

void uav_gimbal_t::state_passive_t::exit(uav_gimbal_t *owner)
{
}

} // namespace pyro
