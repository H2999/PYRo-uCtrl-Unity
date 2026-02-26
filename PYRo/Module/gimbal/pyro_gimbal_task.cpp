#include "pyro_gimbal_task.h"

#include "pyro_dji_motor_drv.h"
#include "pyro_dm_motor_drv.h"

namespace pyro
{
gimbal_t::gimbal_t()
    : module_base_t<gimbal_t, gimbal_cmd_t>("gimbal")
{
    gimbal_ctx = {};
    gimbal_ins = ins_drv_t::get_instance();
    gimbal_ins->init();
    main_fsm.change_state(&state_passive);
}

void gimbal_t::_init()
{
    gimbal_ctx.motor.yaw_motor = new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_1,can_hub_t::can1);
    gimbal_ctx.motor.pitch_motor = new dm_motor_drv_t(0x30,0x40,can_hub_t::can1);
    gimbal_ctx.motor.roll_motor = new dm_motor_drv_t(0x01,0x00,can_hub_t::can1);

    static_cast<dm_motor_drv_t *>(gimbal_ctx.motor.roll_motor)->set_position_range(-PI, PI);
    static_cast<dm_motor_drv_t *>(gimbal_ctx.motor.roll_motor)->set_rotate_range(-30, 30);
    static_cast<dm_motor_drv_t *>(gimbal_ctx.motor.roll_motor)->set_torque_range(-10, 10);

    static_cast<dm_motor_drv_t *>(gimbal_ctx.motor.pitch_motor)->set_position_range(-PI, PI);
    static_cast<dm_motor_drv_t *>(gimbal_ctx.motor.pitch_motor)->set_rotate_range(-20, 20);
    static_cast<dm_motor_drv_t *>(gimbal_ctx.motor.pitch_motor)->set_torque_range(-10, 10);

    gimbal_ctx.pid.yaw_position_pid = new pid_t(5.0f,0.0012,0,1.0f,
                3.0f,200,100,4);
    gimbal_ctx.pid.pitch_position_pid = new pid_t(1.0f,0,0,0.8f,
                3.0f,200,10,4);
    gimbal_ctx.pid.roll_position_pid = new pid_t(1.0f,0,0,3.0f,
                3.0f,200,10,4);

    gimbal_ctx.pid.yaw_speed_pid = new pid_t(1.0f,0.001,0,0.8,
                0.5f,200,100,10);
    gimbal_ctx.pid.pitch_speed_pid = new pid_t(0.5f,0.0008,0,0.8f,
                0.6f,300,100,4);
    gimbal_ctx.pid.roll_speed_pid = new pid_t(0.8f,0,0,3.0f,
                0.5f,200,100,4);
}

void gimbal_t::_update_feedback()
{
    gimbal_ctx.motor.yaw_motor->update_feedback();
    gimbal_ctx.motor.pitch_motor->update_feedback();
    gimbal_ctx.motor.roll_motor->update_feedback();

    gimbal_ins->get_rads_n(&gimbal_ctx.data._current_yaw_angle,
                                          &gimbal_ctx.data._current_pitch_angle,
                                          &gimbal_ctx.data._current_roll_angle);

    gimbal_ins->get_gyro_b(&gimbal_ctx.data._current_yaw_speed,
                                          &gimbal_ctx.data._current_pitch_speed,
                                          &gimbal_ctx.data._current_roll_speed);

    gimbal_ctx.data.correct_yaw_angle = gimbal_ctx.data._current_yaw_angle + gimbal_ctx.data.yaw_offset;

    // gimbal_ctx.data._current_yaw_angle = gimbal_ctx.motor.yaw_motor->get_current_position();
    // gimbal_ctx.data._current_pitch_angle = gimbal_ctx.motor.pitch_motor->get_current_position();
    // gimbal_ctx.data._current_roll_angle = gimbal_ctx.motor.roll_motor->get_current_position();

    // gimbal_ctx.data._current_yaw_speed = gimbal_ctx.motor.yaw_motor->get_current_rotate();
    // gimbal_ctx.data._current_pitch_speed = gimbal_ctx.motor.pitch_motor->get_current_rotate();
    // gimbal_ctx.data._current_roll_speed = gimbal_ctx.motor.roll_motor->get_current_rotate();
}

void gimbal_t::_fsm_execute()
{
    gimbal_ctx.cmd = &_cmd[_read_index];

    if (cmd_base_t::mode_t::ACTIVE == gimbal_ctx.cmd->mode)
        main_fsm.change_state(&state_active);
    else if (cmd_base_t::mode_t::ZERO_FORCE == gimbal_ctx.cmd->mode)
        main_fsm.change_state(&state_passive);

    main_fsm.execute(this);
}

void gimbal_t::gimbal_control(gimbal_ctx_t *ctx)
{
    //注意这里IMU和电机的角度减少的方向是不同的 云台中IMU逆时针是角度减少
    //电机顺时针的时候角度减少 所以要注意目标角度 当前角度 输出扭矩的正负号
    ctx->data._target_yaw_speed = ctx->pid.yaw_position_pid->calculate(
             ctx->data._target_yaw_angle,  ctx->data.correct_yaw_angle);

    ctx->data._target_pitch_speed = ctx->pid.pitch_position_pid->calculate(
            ctx->data._target_pitch_angle,ctx->data._current_pitch_angle);

    ctx->data._target_roll_speed = ctx->pid.roll_position_pid->calculate(
            ctx->data._target_roll_angle,ctx->data._current_roll_angle);

    ctx->data._output_yaw_torque = - ctx->pid.yaw_speed_pid->calculate(
            ctx->data._target_yaw_speed,  ctx->data._current_yaw_speed);

    ctx->data._output_pitch_torque = - ctx->pid.pitch_speed_pid->calculate(
            ctx->data._target_pitch_speed, ctx->data._current_pitch_speed);

    ctx->data._output_roll_torque = - ctx->pid.roll_speed_pid->calculate(
            ctx->data._target_roll_speed, ctx->data._current_roll_speed);
}

void gimbal_t::send_motor_command(const gimbal_ctx_t *ctx)
{
    ctx->motor.yaw_motor->send_torque(ctx->data._output_yaw_torque);

    // ctx->motor.pitch_motor->send_torque(ctx->data._output_pitch_torque);

    // ctx->motor.roll_motor->send_torque(ctx->data._output_roll_torque);
}

void gimbal_t::correct_imu_angle(gimbal_ctx_t *ctx)
{
    float motor_yaw = ctx->motor.yaw_motor->get_current_position();

    ins_drv_t::get_instance()->get_angles_n(&ctx->data.imu_yaw_angle,
        &ctx->data.imu_pitch_angle, &ctx->data.imu_roll_angle);

    float yaw_offset = motor_yaw - ctx->data.imu_yaw_angle;

    if (yaw_offset > PI)
    {
        yaw_offset -= 2.0f * PI;
    }
    else if (yaw_offset < -PI)
    {
        yaw_offset += 2.0f * PI;
    }

    ctx->data.yaw_offset = yaw_offset;
}


}