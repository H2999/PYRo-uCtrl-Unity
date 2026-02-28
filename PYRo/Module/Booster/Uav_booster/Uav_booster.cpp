#include "Uav_booster.h"

#include "pyro_dji_motor_drv.h"

namespace pyro
{
uav_booster_t::uav_booster_t() : module_base_t("quad_booster")
{
    booster_ctx = {};
}

status_t uav_booster_t::_init()
{
    booster_ctx.cfg.motor_cfg.fric_wheel[0] = new dji_m2006_motor_drv_t(dji_motor_tx_frame_t::id_1,can_hub_t::can2);
    booster_ctx.cfg.motor_cfg.fric_wheel[1] = new dji_m2006_motor_drv_t(dji_motor_tx_frame_t::id_2,can_hub_t::can2);

    booster_ctx.cfg.motor_cfg.trigger_wheel = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_3,can_hub_t::can2);

    booster_ctx.cfg.pid_cfg.fric_pid[0] = new pid_t(6.40f, 0.02f, 0.02f, 2.5f, 20, 320, 80, 4);
    booster_ctx.cfg.pid_cfg.fric_pid[1] = new pid_t(6.968f, 0.02f, 0.02f, 2.5f, 20, 320, 80, 4);

    booster_ctx.cfg.pid_cfg.trigger_pos_pid =
        new pid_t(3.4f, 0, 0, 1.0f, 10.0f, 200, 100, 4);
    booster_ctx.cfg.pid_cfg.trigger_spd_pid =
        new pid_t(1.8f, 0, 0.005f, 0, 20.0f, 200, 100, 4);
    return PYRO_OK;
}

void uav_booster_t::_update_feedback()
{
    //更新反馈
    booster_ctx.cfg.motor_cfg.fric_wheel[0]->update_feedback();
    booster_ctx.cfg.motor_cfg.fric_wheel[1]->update_feedback();
    booster_ctx.cfg.motor_cfg.trigger_wheel->update_feedback();

    //获取摩擦轮转速
    booster_ctx.data_ctx.current_fric_speed[0] = booster_ctx.cfg.motor_cfg.fric_wheel[0]->get_current_rotate();
    booster_ctx.data_ctx.current_fric_speed[1] = booster_ctx.cfg.motor_cfg.fric_wheel[1]->get_current_rotate();

    //获取拨弹盘转速 位置和扭矩
    booster_ctx.data_ctx.current_trigger_angle = booster_ctx.cfg.motor_cfg.trigger_wheel->get_current_position();
    booster_ctx.data_ctx.current_trigger_speed = booster_ctx.cfg.motor_cfg.trigger_wheel->get_current_rotate();
    booster_ctx.data_ctx.current_trigger_torque = booster_ctx.cfg.motor_cfg.trigger_wheel->get_current_torque();
}

void uav_booster_t::_fsm_execute()
{
    booster_ctx.cmd = &_current_cmd;

    if (booster_ctx.cmd->mode == cmd_base_t::mode_t::ACTIVE)
        main_fsm.change_state(&active_state);
    else
        main_fsm.change_state(&passive_state);

    main_fsm.execute(this);
}

void uav_booster_t::fric_control()
{
    booster_ctx.data_ctx.fric_output_torque[0] = booster_ctx.cfg.pid_cfg.fric_pid[0]->calculate
        (booster_ctx.data_ctx.target_fric_speed[0], booster_ctx.data_ctx.current_fric_speed[0]);

    booster_ctx.data_ctx.fric_output_torque[1] = booster_ctx.cfg.pid_cfg.fric_pid[1]->calculate
        (booster_ctx.data_ctx.target_fric_speed[1],booster_ctx.data_ctx.current_fric_speed[1]);
}

void uav_booster_t::trigger_position_control()
{
    const float error = booster_ctx.data_ctx.target_trigger_angle - booster_ctx.data_ctx.current_trigger_angle;
    if (error > PI)
    {
        booster_ctx.data_ctx.target_trigger_angle -= 2.0f * PI;
    }
    else if (error < -PI)
    {
        // booster_ctx.data_ctx.target_trig_rad += 2.0f * PI;
    }

    booster_ctx.data_ctx.target_trigger_speed = booster_ctx.cfg.pid_cfg.trigger_pos_pid->calculate
        (booster_ctx.data_ctx.target_trigger_angle, booster_ctx.data_ctx.current_trigger_angle);
}


void uav_booster_t::trigger_speed_control()
{
    booster_ctx.data_ctx.current_trigger_speed = booster_ctx.cfg.pid_cfg.trigger_spd_pid->calculate
        (booster_ctx.data_ctx.target_trigger_speed, booster_ctx.data_ctx.current_trigger_speed);
}

void uav_booster_t::send_fric_command()
{
    booster_ctx.cfg.motor_cfg.fric_wheel[0]->send_torque(booster_ctx.data_ctx.fric_output_torque[0]);
    booster_ctx.cfg.motor_cfg.fric_wheel[1]->send_torque(booster_ctx.data_ctx.fric_output_torque[1]);
}

void uav_booster_t::send_trigger_command()
{
    booster_ctx.cfg.motor_cfg.trigger_wheel->send_torque(booster_ctx.data_ctx.trigger_output_torque);
}

} // namespace pyro
