#include "pyro_uav_shoot.h"

namespace pyro
{
uav_shoot_t::uav_shoot_t()
{
    shoot_ctx = {};
    main_fsm.change_state(&passive_state);
}

void uav_shoot_t::_init()
{
    //初始化电机
    shoot_ctx.motor->fric_motor[0] = new dji_m2006_motor_drv_t(dji_motor_tx_frame_t::id_1
        ,can_hub_t::can2);
    shoot_ctx.motor->fric_motor[1] = new dji_m2006_motor_drv_t(dji_motor_tx_frame_t::id_2
        ,can_hub_t::can2);
    shoot_ctx.motor->trigger_motor = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_3
        ,can_hub_t::can2);

    //初始化摩擦轮pid   摩擦轮用速度环pid
    shoot_ctx.pid->fric_speed_pid[0] = new pid_t(1.0f,0,0.005f,
        5.0f,2.0f,20,0,4);

    shoot_ctx.pid->fric_speed_pid[1] = new pid_t(1.0f,0,0.005f,
     5.0f,2.0f,20,0,4);

    //初始化拨弹盘pid    拨弹盘用位置加速度双环pid
    shoot_ctx.pid->trigger_position_pid = new pid_t(1.0f,0,0.005f,
        5.0f,1.0f,20,0,4);
    shoot_ctx.pid->trigger_speed_pid = new pid_t(1.0f,0,0.005f,
        5.0f,1.0f,20,0,4);
}

void uav_shoot_t::_update_feedback()
{

    shoot_ctx.motor->fric_motor[0]->update_feedback();
    shoot_ctx.motor->fric_motor[1]->update_feedback();
    shoot_ctx.motor->trigger_motor->update_feedback();

    for (uint8_t i = 0; i < 2; i++)
    {
        shoot_ctx.data->current_fric_speed[i] = shoot_ctx.motor->fric_motor[i]->get_current_rotate();
        shoot_ctx.data->output_fric_torque[i] = shoot_ctx.motor->fric_motor[i]->get_current_torque();
    }

    //更新电机反馈
    shoot_ctx.data->current_trigger_angle = shoot_ctx.motor->trigger_motor->get_current_position();
    shoot_ctx.data->current_trigger_speed = shoot_ctx.motor->trigger_motor->get_current_rotate();

    shoot_ctx.data->current_fric_speed[0] = shoot_ctx.motor->fric_motor[0]->get_current_rotate();
    shoot_ctx.data->current_fric_speed[1] = shoot_ctx.motor->fric_motor[1]->get_current_rotate();
}

void uav_shoot_t::_fsm_execute()
{
    shoot_ctx.cmd = &_cmd[_read_index];

    if (cmd_base_t::mode_t::ACTIVE == shoot_ctx.cmd->mode)
        main_fsm.change_state(&active_state);
    else if (cmd_base_t::mode_t::ZERO_FORCE == shoot_ctx.cmd->mode)
        main_fsm.change_state(&passive_state);

    main_fsm.execute(this);
}

void uav_shoot_t::shoot_control(const shoot_ctx_t *cmd)
{
    //摩擦轮的输出扭矩
    cmd->data->output_fric_torque[0] = cmd->pid->fric_speed_pid[0]->calculate
        (cmd->data->target_fric_speed[0],cmd->data->current_fric_speed[0]);
    cmd->data->output_fric_torque[1] = cmd->pid->fric_speed_pid[1]->calculate
        (cmd->data->target_fric_speed[1],cmd->data->current_fric_speed[1]);

    // 拨弹盘的输出扭矩
    cmd->data->target_trigger_speed = cmd->pid->trigger_position_pid->calculate
        (cmd->data->target_trigger_angle,cmd->data->current_trigger_angle);
    cmd->data->output_trigger_torque = cmd->pid->trigger_speed_pid->calculate
        (cmd->data->target_trigger_speed,cmd->data->current_trigger_speed);
}

void uav_shoot_t::send_cammand(const shoot_ctx_t *cmd)
{
    //发送力矩
    cmd->motor->fric_motor[0]->send_torque(cmd->data->output_fric_torque[0]);

    cmd->motor->fric_motor[1]->send_torque(cmd->data->output_fric_torque[1]);

    // cmd->motor->trigger_motor->send_torque(cmd->data->output_trigger_torque);
}


}