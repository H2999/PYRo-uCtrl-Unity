#include "pyro_gimbal_task.h"

#include "pyro_dji_motor_drv.h"
#include "pyro_dm_motor_drv.h"

namespace pyro
{

gimbal_t::gimbal_t()
    : module_base_t<gimbal_t, gimbal_cmd_t>("gimbal", 512, 512,
                                            task_base_t::priority_t::HIGH)

{
    gimbal_ctx = {};
}

void gimbal_t::_init()
{
    gimbal_ctx->motor.yaw_motor = new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_1,can_hub_t::can1);
    gimbal_ctx->motor.pitch_motor = new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_2,can_hub_t::can1);
    gimbal_ctx->motor.roll_motor = new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_3,can_hub_t::can1);

    gimbal_ctx->pid.yaw_pid = new pid_t(5.0f,0.005f,0.001f,0.5f,
                20.0f,20,10,4);
    gimbal_ctx->pid.pitch_pid = new pid_t(5.0f,0.005f,0.001f,0.5f,
                20.0f,20,10,4);
    gimbal_ctx->pid.roll_pid = new pid_t(5.0f,0.005f,0.001f,0.5f,
                20.0f,20,10,4);
}

void gimbal_t::_update_feedback()
{
    gimbal_ctx->motor.yaw_motor->update_feedback();
    gimbal_ctx->motor.pitch_motor->update_feedback();
    gimbal_ctx->motor.roll_motor->update_feedback();

    gimbal_ctx->data._current_yaw_angle = gimbal_ctx->motor.yaw_motor->get_current_position();
    gimbal_ctx->data._current_pitch_angle = gimbal_ctx->motor.pitch_motor->get_current_position();
    gimbal_ctx->data._current_roll_angle = gimbal_ctx->motor.roll_motor->get_current_position();

    gimbal_ctx->data._current_yaw_speed = gimbal_ctx->motor.yaw_motor->get_current_rotate();
    gimbal_ctx->data._current_pitch_speed = gimbal_ctx->motor.pitch_motor->get_current_rotate();
    gimbal_ctx->data._current_roll_speed = gimbal_ctx->motor.roll_motor->get_current_rotate();
}

void gimbal_t::_fsm_execute()
{
    gimbal_ctx->cmd = &_cmd[_read_index];

    if (cmd_base_t::mode_t::ACTIVE == gimbal_ctx->cmd->mode)
        _main_fsm.change_state(&_state_active);
    else if (cmd_base_t::mode_t::ZERO_FORCE == gimbal_ctx->cmd->mode)
        _main_fsm.change_state(&_state_passive);

    _main_fsm.execute(this);
}

void gimbal_t::gimbal_control()
{
    limit_value();
    normalize_angle();

    gimbal_ctx->data._target_yaw_speed = gimbal_ctx->pid.yaw_pid->calculate(
            gimbal_ctx->cmd->_target_yaw_angle,gimbal_ctx->data._current_yaw_angle);

    gimbal_ctx->data._target_pitch_speed = gimbal_ctx->pid.pitch_pid->calculate(
            gimbal_ctx->cmd->_target_pitch_angle,gimbal_ctx->data._current_pitch_angle);

    gimbal_ctx->data._target_roll_speed = gimbal_ctx->pid.roll_pid->calculate(
            gimbal_ctx->cmd->_target_roll_angle,gimbal_ctx->data._current_roll_angle);

    gimbal_ctx->data._output_yaw_torque = gimbal_ctx->pid.yaw_pid->calculate(
            gimbal_ctx->data._target_yaw_speed, gimbal_ctx->data._current_yaw_speed);

    gimbal_ctx->data._output_pitch_torque = gimbal_ctx->pid.pitch_pid->calculate(
            gimbal_ctx->data._target_pitch_speed, gimbal_ctx->data._current_pitch_speed);

    gimbal_ctx->data._output_roll_torque = gimbal_ctx->pid.roll_pid->calculate(
            gimbal_ctx->data._target_roll_speed, gimbal_ctx->data._current_roll_speed);
}

void gimbal_t::send_motor_command()
{
    gimbal_ctx->motor.yaw_motor->send_torque(gimbal_ctx->data._output_yaw_torque);

    gimbal_ctx->motor.pitch_motor->send_torque(gimbal_ctx->data._output_pitch_torque);

    gimbal_ctx->motor.roll_motor->send_torque(gimbal_ctx->data._output_roll_torque);
}

void gimbal_t::limit_value()
{
    if (gimbal_ctx->cmd->_target_yaw_angle > yaw_max_value)
    {
        gimbal_ctx->cmd->_target_yaw_angle = yaw_max_value;
    }
    if (gimbal_ctx->cmd->_target_yaw_angle < yaw_min_value)
    {
        gimbal_ctx->cmd->_target_yaw_angle = yaw_min_value;
    }

    if (gimbal_ctx->cmd->_target_pitch_angle > pitch_max_value)
    {
        gimbal_ctx->cmd->_target_pitch_angle = pitch_max_value;
    }
    if (gimbal_ctx->cmd->_target_pitch_angle < pitch_min_value)
    {
        gimbal_ctx->cmd->_target_pitch_angle = pitch_min_value;
    }

    if (gimbal_ctx->cmd->_target_roll_angle > roll_max_value)
    {
        gimbal_ctx->cmd->_target_roll_angle = roll_max_value;
    }
    if (gimbal_ctx->cmd->_target_roll_angle < roll_min_value)
    {
        gimbal_ctx->cmd->_target_roll_angle = roll_min_value;
    }
}

void gimbal_t::normalize_angle()
{
    if (gimbal_ctx->cmd->_target_yaw_angle > PI)
    {
        gimbal_ctx->cmd->_target_yaw_angle -= 2 * PI;
    }
    if (gimbal_ctx->cmd->_target_yaw_angle < -PI)
    {
        gimbal_ctx->cmd->_target_yaw_angle += 2 * PI;
    }

    if (gimbal_ctx->cmd->_target_pitch_angle > PI)
    {
        gimbal_ctx->cmd->_target_pitch_angle -= 2 * PI;
    }
    if (gimbal_ctx->cmd->_target_pitch_angle < -PI)
    {
        gimbal_ctx->cmd->_target_pitch_angle += 2 * PI;
    }

    if (gimbal_ctx->cmd->_target_roll_angle > PI)
    {
        gimbal_ctx->cmd->_target_roll_angle -= 2 * PI;
    }
    if (gimbal_ctx->cmd->_target_roll_angle < -PI)
    {
        gimbal_ctx->cmd->_target_roll_angle += 2 * PI;
    }
}


}