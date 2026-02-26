#ifndef PYRO_PYRO_UAV_SHOOT_H
#define PYRO_PYRO_UAV_SHOOT_H

#include "pyro_dji_motor_drv.h"
#include "pyro_module_base.h"
#include "pyro_motor_base.h"
#include "pyro_algo_pid.h"
#include "pyro_core_fsm.h"

namespace pyro
{
struct shoot_cmd final : cmd_base_t
{
    bool is_ready;
    bool is_fire;

    float fric_rpm;         // 摩擦轮目标转速
    float trigger_speed;    // 拨弹速度 (转换为角度增量速率)

    shoot_cmd() :is_ready(false), is_fire(false), fric_rpm(0), trigger_speed(0) {}
};

class uav_shoot_t final : public module_base_t<uav_shoot_t,shoot_cmd>
{
    friend class module_base_t;

    struct motor_ctx_t;
    struct pid_ctx_t;
    struct data_ctx_t;
    struct shoot_ctx_t;
    //判断是否卡弹的结构体
    struct check_block;

public:
    uav_shoot_t(const uav_shoot_t &)            = delete;
    uav_shoot_t &operator=(const uav_shoot_t &) = delete;

private:
    uav_shoot_t();
    ~uav_shoot_t() override = default;

    //重写回调函数
    void _init() override;
    void _update_feedback() override;
    void _fsm_execute() override;

    //派生方法
    static void shoot_control(const shoot_ctx_t *cmd);
    static void send_cammand(const shoot_ctx_t *cmd);

    struct motor_ctx_t
    {
        motor_base_t *fric_motor[2]{nullptr};
        motor_base_t *trigger_motor{nullptr};
    };

    struct pid_ctx_t
    {
        pid_t *trigger_position_pid{nullptr};
        pid_t *trigger_speed_pid{nullptr};

        pid_t *fric_speed_pid[2]{nullptr};
    };

    struct data_ctx_t
    {
        //拨弹盘目标角度 目标角速度
        float target_trigger_angle{};
        float target_trigger_speed{};
        //拨弹盘当前角度 当前角速度
        float current_trigger_angle{};
        float current_trigger_speed{};
        float current_trig_torque{};

        //摩擦轮目标速度
        float target_fric_speed[2]{};i
        //摩擦轮当前速度
        float current_fric_speed[2]{};

        // 摩擦轮和拨弹盘的输出扭矩
        float output_fric_torque[2]{};
        float output_trigger_torque{};
    };

    struct check_block
    {
        bool block_flag{false};
        uint16_t block_time{0};
        uint16_t adjust_time{0};
    };

    struct shoot_ctx_t
    {
        motor_ctx_t *motor{nullptr};
        pid_ctx_t   *pid{nullptr};
        data_ctx_t  *data{nullptr};
        shoot_cmd   *cmd{nullptr};
        check_block *is_block{nullptr};
    };

    shoot_ctx_t shoot_ctx{};

    struct passive_state_t final : state_t<uav_shoot_t>
    {
        void enter(uav_shoot_t *owner) override;
        void execute(uav_shoot_t *owner) override;
        void exit(uav_shoot_t *owner) override;
    };

    struct fsm_active_state : fsm_t<uav_shoot_t>
    {
        struct ready_state_t final : state_t<uav_shoot_t>
        {
            void enter(uav_shoot_t *owner) override;
            void execute(uav_shoot_t *owner) override;
            void exit(uav_shoot_t *owner) override;
        };

        struct fire_state_t final : state_t<uav_shoot_t>
        {
            void enter(uav_shoot_t *owner) override;
            void execute(uav_shoot_t *owner) override;
            void exit(uav_shoot_t *owner) override;
        };

        void on_enter(uav_shoot_t *owner) override;
        void on_execute(uav_shoot_t *owner) override;
        void on_exit(uav_shoot_t *owner) override;

    private:
        ready_state_t ready_state;
        fire_state_t fire_state;
    };


    passive_state_t passive_state{};
    fsm_active_state active_state;
    fsm_t<uav_shoot_t> main_fsm;

    static constexpr uint16_t block_time = 200;
    static constexpr uint16_t adjust_time = 200;
};
}
#endif // PYRO_PYRO_UAV_SHOOT_H