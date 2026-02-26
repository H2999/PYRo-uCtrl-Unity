#include "Gimbal/Uav/pyro_uav_gimbal.h"
namespace pyro
{

void gimbal_t::state_passive_t::enter(gimbal_t *owner)
{
    owner->gimbal_ctx.gimbal_cfg.pid_cfg->yaw_position_pid->clear();
    owner->gimbal_ctx.gimbal_cfg.pid_cfg->pitch_position_pid->clear();
    owner->gimbal_ctx.gimbal_cfg.pid_cfg->roll_position_pid->clear();

    owner->gimbal_ctx.gimbal_cfg.pid_cfg->yaw_speed_pid->clear();
    owner->gimbal_ctx.gimbal_cfg.pid_cfg->pitch_speed_pid->clear();
    owner->gimbal_ctx.gimbal_cfg.pid_cfg->roll_speed_pid->clear();

    owner->gimbal_ctx.gimbal_cfg.motor_cfg->yaw_motor->disable();
    owner->gimbal_ctx.gimbal_cfg.motor_cfg->pitch_motor->disable();
    owner->gimbal_ctx.gimbal_cfg.motor_cfg->roll_motor->disable();

    correct_imu_angle(&owner->gimbal_ctx);
}

void gimbal_t::state_passive_t::execute(gimbal_t *owner)
{
    owner->gimbal_ctx.data_ctx._output_yaw_torque = 0;
    owner->gimbal_ctx.data_ctx._output_pitch_torque = 0;
    owner->gimbal_ctx.data_ctx._output_roll_torque = 0;

    owner->gimbal_ctx.data_ctx._target_yaw_angle = owner->gimbal_ctx.data_ctx._current_yaw_angle;
    owner->gimbal_ctx.data_ctx._target_pitch_angle = owner->gimbal_ctx.data_ctx._current_pitch_angle;
    owner->gimbal_ctx.data_ctx._target_roll_angle = owner->gimbal_ctx.data_ctx._current_roll_angle;

    owner->gimbal_ctx.data_ctx._target_yaw_speed = 0;
    owner->gimbal_ctx.data_ctx._target_pitch_speed = 0;
    owner->gimbal_ctx.data_ctx._target_roll_speed = 0;

    send_motor_command(&owner->gimbal_ctx);
}

void gimbal_t::state_passive_t::exit(gimbal_t *owner)
{
}

} // namespace pyro
