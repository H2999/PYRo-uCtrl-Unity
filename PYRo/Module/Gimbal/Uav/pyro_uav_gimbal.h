//
// Created by 1 on 2026/2/25.
//

#ifndef PYRO_UAV_H
#define PYRO_UAV_H

#include "pyro_algo_pid.h"
#include "pyro_core_fsm.h"
#include "pyro_ins.h"
#include "pyro_module_base.h"
#include "pyro_motor_base.h"

namespace pyro
{
//命令定义
struct gimbal_cmd_t final : cmd_base_t
{
    float yaw_delta_angle;
    float pitch_delta_angle;
    float roll_delta_angle;

    gimbal_cmd_t()
    :yaw_delta_angle() , pitch_delta_angle(0) , roll_delta_angle(0)
    {
    }
};

//配置文件 初始化的时候赋值
struct gimbal_cfg_t
{
    struct motor_cfg_t
    {
        motor_base_t *yaw_motor{nullptr};
        motor_base_t *pitch_motor{nullptr};
        motor_base_t *roll_motor{nullptr};
    };

    struct pid_cfg_t
    {
        pid_t *yaw_position_pid{nullptr};
        pid_t *pitch_position_pid{nullptr};
        pid_t *roll_position_pid{nullptr};

        pid_t *yaw_speed_pid{nullptr};
        pid_t *pitch_speed_pid{nullptr};
        pid_t *roll_speed_pid{nullptr};
    };

    motor_cfg_t *motor_cfg{};
    pid_cfg_t *pid_cfg{};

    float yaw_offset{};
};

//云台类定义
class gimbal_t final : public module_base_t<gimbal_t,gimbal_cmd_t,gimbal_cfg_t>
{
    friend class module_base_t;
    friend class jcom_drv_t;

    struct data_ctx_t;
    struct gimbal_ctx_t;
    struct correct_imu_ctx_t;

public:
    gimbal_t(const gimbal_t &) = delete;
    gimbal_t & operator=(const gimbal_t &) = delete;

private:
    gimbal_t();
    ~gimbal_t() override = default;

    //基类接口
    status_t _init() override;
    void _update_feedback() override;
    void _fsm_execute() override;

    struct data_ctx_t
    {
        //目标角度
        float _target_yaw_angle{};
        float _target_pitch_angle{};
        float _target_roll_angle{};
        //目标速度
        float _target_yaw_speed{};
        float _target_pitch_speed{};
        float _target_roll_speed{};
        //当前角度
        float _current_yaw_angle{};
        float _current_pitch_angle{};
        float _current_roll_angle{};
        // 当前速度
        float _current_pitch_speed{};
        float _current_yaw_speed{};
        float _current_roll_speed{};
        // 输出扭矩
        float _output_yaw_torque{};
        float _output_pitch_torque{};
        float _output_roll_torque{};
    };

    struct correct_imu_ctx_t
    {
        //矫正后的角度
        float correct_yaw_angle{};
        float correct_pitch_angle{};
        float correct_roll_angle{};

        float imu_yaw_angle{};
        float imu_pitch_angle{};
        float imu_roll_angle{};
    };


    struct gimbal_ctx_t
    {
        gimbal_cfg_t gimbal_cfg{};
        data_ctx_t data_ctx{};
        correct_imu_ctx_t correct_imu_ctx{};
        gimbal_cmd_t *cmd{};
    };

    gimbal_ctx_t gimbal_ctx{};
    ins_drv_t *gimbal_ins;

    //派生方法
    static void gimbal_control(gimbal_ctx_t *ctx);
    static void send_motor_command(gimbal_ctx_t *ctx);
    static void correct_imu_angle(gimbal_ctx_t *ctx);

    struct state_passive_t final : state_t<gimbal_t>
    {
        void enter(gimbal_t *owner) override;
        void execute(gimbal_t *owner) override;
        void exit(gimbal_t *owner) override;
    };

    struct state_active_t final : state_t<gimbal_t>
    {
        void enter(gimbal_t *owner) override;
        void execute(gimbal_t *owner) override;
        void exit(gimbal_t *owner) override;
    };

    // 状态实例
    state_passive_t state_passive;
    state_active_t state_active;
    fsm_t<gimbal_t> main_fsm;

    static constexpr float yaw_max_value = 2.3f;
    static constexpr float yaw_min_value = -0.8;

    static constexpr float pitch_max_value = 0.23f;
    static constexpr float pitch_min_value = -0.35f;

    static constexpr float roll_max_value = 0.2f;
    static constexpr float roll_min_value = -0.45f;
};

}

#endif // PYRO_UAV_H
