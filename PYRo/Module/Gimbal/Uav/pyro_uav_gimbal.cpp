#include "pyro_uav_gimbal.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_ins.h"

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
    gimbal_ctx.motor.yaw_motor = new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_5,can_hub_t::can1);
    gimbal_ctx.motor.pitch_motor = new dm_motor_drv_t(0x11,0x10,can_hub_t::can1);
    gimbal_ctx.motor.roll_motor = new dm_motor_drv_t(0x01,0x00,can_hub_t::can1);

    static_cast<dm_motor_drv_t *>(gimbal_ctx.motor.roll_motor)->set_position_range(-PI, PI);
    static_cast<dm_motor_drv_t *>(gimbal_ctx.motor.roll_motor)->set_rotate_range(-30, 30);
    static_cast<dm_motor_drv_t *>(gimbal_ctx.motor.roll_motor)->set_torque_range(-10, 10);

    static_cast<dm_motor_drv_t *>(gimbal_ctx.motor.pitch_motor)->set_position_range(-PI, PI);
    static_cast<dm_motor_drv_t *>(gimbal_ctx.motor.pitch_motor)->set_rotate_range(-15.333333, 15.333333);
    static_cast<dm_motor_drv_t *>(gimbal_ctx.motor.pitch_motor)->set_torque_range(-3, 3);

    gimbal_ctx.pid.yaw_position_pid = new pid_t(12.4f,0.0012f,0,2.0f,
                4.5f,200,100,4);
    gimbal_ctx.pid.pitch_position_pid = new pid_t(5.125f,0.0008f,0.0f,1.0f,
                3.0f,200,90,4);
    gimbal_ctx.pid.roll_position_pid = new pid_t(5.8f,0.0f,0.0f,2.0f,
                3.0f,200,120,4);

    gimbal_ctx.pid.yaw_speed_pid = new pid_t(0.95f,0.009f,0.0008f,1.8f,
                3.0f,230,120,4);
    gimbal_ctx.pid.pitch_speed_pid = new pid_t(1.05f,0.018f,0.0012f,2.5f,
                8.0f,230,100,4);
    gimbal_ctx.pid.roll_speed_pid = new pid_t(1.28f,0.008f,0.0004f,1.8f,
                10.0f,200,120,4);

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

    gimbal_ctx.data.yaw_motor_angle = gimbal_ctx.motor.yaw_motor->get_current_position();
    gimbal_ctx.data.pitch_motor_angle = gimbal_ctx.motor.pitch_motor->get_current_position();
    gimbal_ctx.data.roll_motor_angle = gimbal_ctx.motor.roll_motor->get_current_position();
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
    //由于是在刚上力的时候读取电机的角度当作偏移量 但是上力之后就会循环进行gimbal_control和send_motor_command的指令 导致刚上力的一瞬间目标值和矫正值不相等
    //让电机抽动一下 所以加一个标志位 在刚上力的时候改一下标志位 然后检测标志位让目标值和校正值相等防止电机抽动
    if (ctx->data.correct_imu_ctx.correct_flag)
    {
        ctx->data.correct_imu_ctx.correct_yaw_angle = - ctx->data._current_yaw_angle + ctx->data.correct_imu_ctx.yaw_offset;
        ctx->data.correct_imu_ctx.correct_pitch_angle = - ctx->data._current_pitch_angle;
        ctx->data.correct_imu_ctx.correct_roll_angle = ctx->data._current_roll_angle;

        ctx->data._target_yaw_angle = ctx->data.correct_imu_ctx.correct_yaw_angle;
        ctx->data._target_pitch_angle = ctx->data.correct_imu_ctx.correct_pitch_angle;
        ctx->data.final_roll_angle = ctx->data.correct_imu_ctx.correct_roll_angle;
        ctx->data.correct_imu_ctx.correct_flag = 0;
    }
    //注意这里 IMU和电机的角度减少的方向是不同的 云台中IMU逆时针是角度减少
    //电机顺时针的时候角度减少 所以要注意目标角度 当前角度 输出扭矩的正负号

    //用IMU读取出来的角度加上偏移量得到 加上offset的角度
    ctx->data.correct_imu_ctx.correct_yaw_angle = - ctx->data._current_yaw_angle + ctx->data.correct_imu_ctx.yaw_offset;

    //pitch轴和roll轴有重力辅助矫正角度 读出来的数据就是和用电机读出来的数据相同
    ctx->data.correct_imu_ctx.correct_pitch_angle = - ctx->data._current_pitch_angle;

    //roll轴 imu减小的方向和电机减小的方向相同 所以不用加负号了
    ctx->data.correct_imu_ctx.correct_roll_angle = ctx->data._current_roll_angle;

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
            ctx->data.final_roll_angle,ctx->data.correct_imu_ctx.correct_roll_angle);

    //读取出来的速度和电机读取出来的也是反向的 所以也要加个负号 这样最后算出来的输出扭矩才是正确的
    ctx->data._output_yaw_torque = ctx->pid.yaw_speed_pid->calculate(
            ctx->data._target_yaw_speed,- ctx->data._current_yaw_speed);

    ctx->data._output_pitch_torque = ctx->pid.pitch_speed_pid->calculate(
            ctx->data._target_pitch_speed,- ctx->data._current_pitch_speed);

    ctx->data._output_roll_torque = ctx->pid.roll_speed_pid->calculate(
            ctx->data._target_roll_speed,ctx->data._current_roll_speed);
}

void uav_gimbal_t::send_motor_command(const gimbal_ctx_t *ctx)
{
     ctx->motor.yaw_motor->send_torque(ctx->data._output_yaw_torque);

     ctx->motor.pitch_motor->send_torque(ctx->data._output_pitch_torque);

     ctx->motor.roll_motor->send_torque(ctx->data._output_roll_torque);
}

void uav_gimbal_t::normalize_angle(float& angle) {
    if (angle > PI)
    {
        angle -= 2.0f * PI;
    }
    else if (angle < -PI)
    {
        angle += 2.0f * PI;
    }
}

}