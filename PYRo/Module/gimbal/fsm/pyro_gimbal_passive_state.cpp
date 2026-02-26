
#include "pyro_gimbal_task.h"
namespace pyro
{

void gimbal_t::state_passive_t::enter(gimbal_t *owner)
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

    correct_imu_angle(&owner->gimbal_ctx);
}

void gimbal_t::state_passive_t::execute(gimbal_t *owner)
{
    owner->gimbal_ctx.data._output_yaw_torque = 0;
    owner->gimbal_ctx.data._output_pitch_torque = 0;
    owner->gimbal_ctx.data._output_roll_torque = 0;

    owner->gimbal_ctx.data._target_yaw_angle = owner->gimbal_ctx.data._current_yaw_angle;
    owner->gimbal_ctx.data._target_pitch_angle = owner->gimbal_ctx.data._current_pitch_angle;
    owner->gimbal_ctx.data._target_roll_angle = owner->gimbal_ctx.data._current_roll_angle;

    owner->gimbal_ctx.data._target_yaw_speed = 0;
    owner->gimbal_ctx.data._target_pitch_speed = 0;
    owner->gimbal_ctx.data._target_roll_speed = 0;

    send_motor_command(&owner->gimbal_ctx);
}

void gimbal_t::state_passive_t::exit(gimbal_t *owner)
{
}

} // namespace pyro