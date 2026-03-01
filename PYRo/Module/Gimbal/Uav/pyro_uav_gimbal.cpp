#include "pyro_dji_motor_drv.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_ins.h"
#include "pyro_uav_gimbal.h"

namespace pyro
{
uav_gimbal_t::uav_gimbal_t()
    : module_base_t("gimbal")
{
    gimbal_ctx = {};
    gimbal_ins = ins_drv_t::get_instance();
    gimbal_ins->init();
}

status_t uav_gimbal_t::_init()
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

    gimbal_ctx.pid.yaw_position_pid = new pid_t(12.5f,0.0f,0.0f,1.0f,
                3.0f,200,100,4);
    gimbal_ctx.pid.pitch_position_pid = new pid_t(4.5f,0.0f,0.0f,1.0f,
                3.0f,200,10,4);
    gimbal_ctx.pid.roll_position_pid = new pid_t(3.8f,0.0005f,0.0007f,3.0f,
                3.0f,200,10,4);

    gimbal_ctx.pid.yaw_speed_pid = new pid_t(0.85f,0.0085f,0.0005f,0.8f,
                0.5f,260,150,4);
    gimbal_ctx.pid.pitch_speed_pid = new pid_t(0.4f,0.0008f,0.0008f,2.0f,
                3.0f,230,180,4);
    gimbal_ctx.pid.roll_speed_pid = new pid_t(0.5f,0.0004f,0.0f,0.8f,
                3.0f,200,100,4);

    return PYRO_OK;
}

void uav_gimbal_t::_update_feedback()
{
    gimbal_ctx.motor.yaw_motor->update_feedback();
    gimbal_ctx.motor.pitch_motor->update_feedback();
    gimbal_ctx.motor.roll_motor->update_feedback();

    //读取IMU获得当前角度
    gimbal_ins->get_rads_n(&gimbal_ctx.data._current_yaw_angle,
                                          &gimbal_ctx.data._current_pitch_angle,
                                          &gimbal_ctx.data._current_roll_angle);

    gimbal_ins->get_gyro_b(&gimbal_ctx.data._current_yaw_speed,
                                          &gimbal_ctx.data._current_pitch_speed,
                                          &gimbal_ctx.data._current_roll_speed);

    gimbal_ctx.data.pitch_motor_angle = gimbal_ctx.motor.pitch_motor->get_current_position();
    gimbal_ctx.data.roll_motor_angle = gimbal_ctx.motor.roll_motor->get_current_position();
    gimbal_ctx.data.yaw_motor_angle = gimbal_ctx.motor.yaw_motor->get_current_position();

    // gimbal_ctx.data._current_yaw_angle = gimbal_ctx.motor.yaw_motor->get_current_position();
    //gimbal_ctx.data._current_pitch_angle = gimbal_ctx.motor.pitch_motor->get_current_position();
    //gimbal_ctx.data._current_roll_angle = gimbal_ctx.motor.roll_motor->get_current_position();

    // gimbal_ctx.data._current_yaw_speed = gimbal_ctx.motor.yaw_motor->get_current_rotate();
    //gimbal_ctx.data._current_pitch_speed = gimbal_ctx.motor.pitch_motor->get_current_rotate();
    //gimbal_ctx.data._current_roll_speed = gimbal_ctx.motor.roll_motor->get_current_rotate();
}

void uav_gimbal_t::_fsm_execute()
{
    gimbal_ctx.cmd = &_current_cmd;

    if (cmd_base_t::mode_t::ACTIVE == gimbal_ctx.cmd->mode)
        main_fsm.change_state(&state_active);
    else if (cmd_base_t::mode_t::PASSIVE == gimbal_ctx.cmd->mode)
        main_fsm.change_state(&state_passive);

    main_fsm.execute(this);
}

void uav_gimbal_t::gimbal_control(gimbal_ctx_t *ctx)
{
    //注意这里 IMU和电机的角度减少的方向是不同的 云台中IMU逆时针是角度减少
    //电机顺时针的时候角度减少 所以要注意目标角度 当前角度 输出扭矩的正负号

    //用IMU读取出来的角度加上偏移量得到 加上offset的角度
    ctx->data.correct_imu_ctx.correct_yaw_angle = - ctx->data._current_yaw_angle + ctx->data.correct_imu_ctx.yaw_offset;

    //pitch轴和roll轴有重力辅助矫正角度 读出来的数据就是和用电机读出来的数据相同
    ctx->data.correct_imu_ctx.correct_pitch_angle = - ctx->data._current_pitch_angle;
    ctx->data.correct_imu_ctx.correct_roll_angle = - ctx->data._current_roll_angle;

    //对角度进行归一化
    if (ctx->data.correct_imu_ctx.correct_yaw_angle > PI)
    {
        ctx->data.correct_imu_ctx.correct_yaw_angle -= 2.0f * PI;
    }
    else if (ctx->data.correct_imu_ctx.correct_yaw_angle < -PI)
    {
        ctx->data.correct_imu_ctx.correct_yaw_angle += 2.0f * PI;
    }

    ctx->data._target_yaw_speed = ctx->pid.yaw_position_pid->calculate(
             ctx->data._target_yaw_angle, ctx->data.correct_imu_ctx.correct_yaw_angle);

    ctx->data._target_pitch_speed = ctx->pid.pitch_position_pid->calculate(
            ctx->data._target_pitch_angle, ctx->data.correct_imu_ctx.correct_pitch_angle);

    ctx->data._target_roll_speed = ctx->pid.roll_position_pid->calculate(
            ctx->data._target_roll_angle,ctx->data.correct_imu_ctx.correct_roll_angle);

    //读取出来的速度和电机读取出来的也是反向的 所以也要加个负号 这样最后算出来的输出扭矩才是正确的
    ctx->data._output_yaw_torque = ctx->pid.yaw_speed_pid->calculate(
            ctx->data._target_yaw_speed,- ctx->data._current_yaw_speed);

    ctx->data._output_pitch_torque = ctx->pid.pitch_speed_pid->calculate(
            ctx->data._target_pitch_speed,- ctx->data._current_pitch_speed);

    // if (ctx->data._target_pitch_angle < 0)
    // {
    //     ctx->data._output_pitch_torque += 0.005f;
    // }

    ctx->data._output_roll_torque = ctx->pid.roll_speed_pid->calculate(
            ctx->data._target_roll_speed,- ctx->data._current_roll_speed);
}

void uav_gimbal_t::send_motor_command(const gimbal_ctx_t *ctx)
{
     ctx->motor.yaw_motor->send_torque(ctx->data._output_yaw_torque);

     // ctx->motor.pitch_motor->send_torque(ctx->data._output_pitch_torque);

     // ctx->motor.roll_motor->send_torque(ctx->data._output_roll_torque);
}

}