#ifndef PYRO_GIMBAL_TASK_H
#define PYRO_GIMBAL_TASK_H

#include "pyro_algo_pid.h"
#include "pyro_core_fsm.h"
#include "pyro_module_base.h"
#include "pyro_motor_base.h"

namespace pyro
{
//命令定义
struct gimbal_cmd_t final : cmd_base_t
{
    float _target_yaw_angle;
    float _target_pitch_angle;
    float _target_roll_angle;

    gimbal_cmd_t()
    :_target_yaw_angle() , _target_pitch_angle(0) , _target_roll_angle(0)
    {
    }
};

//云台类定义
class gimbal_t :public module_base_t<gimbal_t,gimbal_cmd_t>
{
    friend class module_base_t;
    friend class jcom_drv_t;

    struct motor_ctx_t;
    struct pid_ctx_t;
    struct data_ctx_t;
    struct gimbal_ctx_t;

public:
    gimbal_t(const gimbal_t &)            = delete;
    gimbal_t &operator=(const gimbal_t &) = delete;

private:
    gimbal_t();
    ~gimbal_t() override = default;

    //基类接口
    void _init() override;
    void _update_feedback() override;
    void _fsm_execute() override;

    //派生方法
    void gimbal_control();
    void send_motor_command();
    void limit_value();
    void normalize_angle();

    struct motor_ctx_t
    {
        motor_base_t *yaw_motor{nullptr};
        motor_base_t *pitch_motor{nullptr};
        motor_base_t *roll_motor{nullptr};
    };

    struct pid_ctx_t
    {
        pid_t *yaw_pid{nullptr};
        pid_t *pitch_pid{nullptr};
        pid_t *roll_pid{nullptr};
    };

    struct data_ctx_t
    {
        float _target_yaw_speed{};
        float _target_pitch_speed{};
        float _target_roll_speed{};

        float _current_yaw_angle{};
        float _current_pitch_angle{};
        float _current_roll_angle{};

        float _current_pitch_speed{};
        float _current_yaw_speed{};
        float _current_roll_speed{};

        float _output_yaw_torque{};
        float _output_pitch_torque{};
        float _output_roll_torque{};
    };

    struct gimbal_ctx_t
    {
        motor_ctx_t motor;
        pid_ctx_t pid;
        data_ctx_t data{};
        gimbal_cmd_t *cmd{};
    };

    // struct debug_ctx_t
    // {
    //     float debug_leg_torque[2]{};
    // };

    gimbal_ctx_t *gimbal_ctx;
    // debug_ctx_t debug_data;

    struct state_passive_t final : public state_t<gimbal_t>
    {
        void enter(gimbal_t *owner) override;
        void execute(gimbal_t *owner) override;
        void exit(gimbal_t *owner) override;
    };

    struct fsm_active_t final : public fsm_t<gimbal_t>
    {
        struct state_motion_t final : public state_t<gimbal_t>
        {
            void enter(gimbal_t *owner) override;
            void execute(gimbal_t *owner) override;
            void exit(gimbal_t *owner) override;
        };


        // FSM Hooks
        void on_enter(gimbal_t *owner) override;
        void on_execute(gimbal_t *owner) override;
        void on_exit(gimbal_t *owner) override;
    private:
        state_motion_t _state_motion;
    };


    // 状态实例
    state_passive_t _state_passive;
    fsm_active_t _state_active;
    fsm_t<gimbal_t> _main_fsm;

    static constexpr float yaw_max_value = 1.0f;
    static constexpr float yaw_min_value = -1.0f;

    static constexpr float pitch_max_value = 1.0f;
    static constexpr float pitch_min_value = -1.0f;

    static constexpr float roll_max_value = 1.0f;
    static constexpr float roll_min_value = -1.0f;
};
}

#endif
