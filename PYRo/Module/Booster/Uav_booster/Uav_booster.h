#ifndef PYRO_UAV_BOOSTER_H
#define PYRO_UAV_BOOSTER_H

#include "pyro_algo_pid.h"
#include "pyro_module_base.h"
#include "pyro_motor_base.h"

namespace pyro
{

struct uav_booster_cmd_t final : public cmd_base_t
{
    bool fric_on;     // 摩擦轮开启
    bool trigger_enable; // 拨弹开启
    float target_fric1_speed;  // 第一级摩擦轮目标转速
    float target_fric2_speed;  // 第二级摩擦轮目标转速

    uav_booster_cmd_t()
        : fric_on(false), trigger_enable(false), target_fric1_speed(0), target_fric2_speed(0)
    {
    }
};

struct uav_booster_cfg_t
{
    struct motor_cfg_t
    {
        motor_base_t *fric_wheel[2]{nullptr};
        motor_base_t *trigger_wheel{nullptr};
    };

    struct pid_cfg_t
    {
        pid_t *fric_pid[2]{nullptr};
        pid_t *trigger_pos_pid{nullptr};
        pid_t *trigger_spd_pid{nullptr};
    };

    motor_cfg_t motor_cfg;
    pid_cfg_t pid_cfg;
};

class uav_booster_t : public pyro::module_base_t<uav_booster_t,uav_booster_cmd_t,uav_booster_cfg_t>
{
    friend class module_base_t<uav_booster_t, uav_booster_cmd_t, uav_booster_cfg_t>;
    friend class jcom_drv_t;

    struct data_ctx_t;
    struct booster_ctx_t;

public:
    uav_booster_t(const uav_booster_t &) = delete;
    uav_booster_t & operator = (const uav_booster_t &) = delete;

private:
    uav_booster_t();
    ~uav_booster_t() override = default;

    //接口
    status_t _init() override;
    void _update_feedback() override;
    void _fsm_execute() override;

    //派生方法
    void fric_control();
    void trigger_position_control();
    void trigger_speed_control();
    void send_fric_command();
    void send_trigger_command();

    struct data_ctx_t
    {
        float current_fric_speed[2]{};
        float current_trigger_angle{0};
        float current_trigger_speed{};
        float current_trigger_torque{0};

        float target_fric_speed[2]{};
        float target_trigger_angle{0};
        float target_trigger_speed{0};

        float fric_output_torque[2]{};
        float trigger_output_torque{0};
    };

    struct booster_ctx_t
    {
        uav_booster_cfg_t cfg;
        data_ctx_t data_ctx;
        uav_booster_cmd_t *cmd{};
    };

    booster_ctx_t booster_ctx;

    struct passive_state_t final : public state_t<uav_booster_t>
    {
        void enter(uav_booster_t *owner) override;
        void execute(uav_booster_t *owner) override;
        void exit(uav_booster_t *owner) override;

    private:
        bool trigger_stopped{false}; // 用于确保拨弹盘完全停止后发0
    };

    struct fsm_active_t final : public fsm_t<uav_booster_t>
    {
        // struct state_homing_t final : public state_t<owner>
        // {
        //     void enter(uav_booster_t *owner) override;
        //     void execute(uav_booster_t *owner) override;
        //     void exit(uav_booster_t *owner) override;
        //
        // private:
        //     float _homing_turnback_start_time{0.0f};
        // };
        // struct state_interim_t final : public state_t<uav_booster_t>
        // {
        //     void enter(uav_booster_t *owner) override;
        //     void execute(uav_booster_t *owner) override;
        //     void exit(uav_booster_t *owner) override;
        // };
        struct ready_state_t final : public state_t<uav_booster_t>
        {
            void enter(uav_booster_t *owner) override;
            void execute(uav_booster_t *owner) override;
            void exit(uav_booster_t *owner) override;
        };
        struct firing_state_t final : public state_t<uav_booster_t>
        {
            void enter(uav_booster_t *owner) override;
            void execute(uav_booster_t *owner) override;
            void exit(uav_booster_t *owner) override;
        };
        struct stall_state_t final : public state_t<uav_booster_t>
        {
            void enter(uav_booster_t *owner) override;
            void execute(uav_booster_t *owner) override;
            void exit(uav_booster_t *owner) override;
        };
        void on_enter(uav_booster_t *owner) override;
        void on_execute(uav_booster_t *owner) override;
        void on_exit(uav_booster_t *owner) override;

    private:
        // state_homing_t _homing_state;
        // state_interim_t _interim_state;
        ready_state_t ready_state;
        firing_state_t busy_state;
        stall_state_t stall_state;
    };
    passive_state_t passive_state;
    fsm_active_t active_state;
    fsm_t<uav_booster_t> main_fsm;
};

};

#endif // PYRO_UAV_BOOSTER_H
