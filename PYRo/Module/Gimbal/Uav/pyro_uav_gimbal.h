#ifndef PYRO_GIMBAL_TASK_H
#define PYRO_GIMBAL_TASK_H

#include "pyro_algo_pid.h"
#include "pyro_core_fsm.h"
#include "pyro_ins.h"
#include "pyro_module_base.h"
#include "pyro_motor_base.h"

namespace pyro
{
//命令定义
struct uav_gimbal_cmd_t final : cmd_base_t
{
    float yaw_delta_angle;      //yaw轴目标角度
    float pitch_delta_angle;    //pitch轴目标角度
    float roll_delta_angle;     //roll轴目标角度

    uav_gimbal_cmd_t()
    :yaw_delta_angle() , pitch_delta_angle(0) , roll_delta_angle(0)
    {
    }
};

struct uav_gimbal_cfg_t{};

//云台类定义
class uav_gimbal_t final : public module_base_t<uav_gimbal_t,uav_gimbal_cmd_t,uav_gimbal_cfg_t>
{
    friend class module_base_t;
    friend class jcom_drv_t;

    struct motor_ctx_t;
    struct pid_ctx_t;
    struct data_ctx_t;
    struct gimbal_ctx_t;

public:
    uav_gimbal_t(const uav_gimbal_t &)            = delete;
    uav_gimbal_t &operator=(const uav_gimbal_t &) = delete;

private:
    uav_gimbal_t();
    ~uav_gimbal_t() override = default;

    //基类接口
    status_t _init() override;
    void _update_feedback() override;
    void _fsm_execute() override;

    //派生方法
    static void gimbal_control(gimbal_ctx_t *ctx);
    static void send_motor_command(const gimbal_ctx_t *ctx);
    static void normalize_angle(float& angle);

    struct motor_ctx_t
    {
        motor_base_t *yaw_motor{nullptr};
        motor_base_t *pitch_motor{nullptr};
        motor_base_t *roll_motor{nullptr};
    };

    struct pid_ctx_t
    {
        pid_t *yaw_position_pid{nullptr};
        pid_t *pitch_position_pid{nullptr};
        pid_t *roll_position_pid{nullptr};

        pid_t *yaw_speed_pid{nullptr};
        pid_t *pitch_speed_pid{nullptr};
        pid_t *roll_speed_pid{nullptr};
    };

    struct data_ctx_t
    {
        //目标角度
        float _target_yaw_angle{};
        float _target_pitch_angle{};
        float _target_roll_angle{};
        float final_roll_angle{};
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

        float pitch_motor_angle{};
        float roll_motor_angle{};
        float yaw_motor_angle{};

        struct correct_imu_ctx_t
        {
            float yaw_offset{};
            uint8_t correct_flag{};

            float correct_yaw_angle{};
            float correct_pitch_angle{};
            float correct_roll_angle{};
        };

        correct_imu_ctx_t correct_imu_ctx;
    };

    struct gimbal_ctx_t
    {
        motor_ctx_t motor;
        pid_ctx_t pid;
        data_ctx_t data{};
        uav_gimbal_cmd_t *cmd{};
    };

    gimbal_ctx_t gimbal_ctx;
    ins_drv_t *gimbal_ins;

    struct state_passive_t final : public state_t<uav_gimbal_t>
    {
        void enter(uav_gimbal_t *owner) override;
        void execute(uav_gimbal_t *owner) override;
        void exit(uav_gimbal_t *owner) override;
    };

    struct state_active_t final : public state_t<uav_gimbal_t>
    {
        void enter(uav_gimbal_t *owner) override;
        void execute(uav_gimbal_t *owner) override;
        void exit(uav_gimbal_t *owner) override;
    };

    // 状态实例
    state_passive_t state_passive;
    state_active_t state_active;
    fsm_t<uav_gimbal_t> main_fsm;

    static constexpr float yaw_max_value = 1.6f;
    static constexpr float yaw_min_value = -2.1f;

    //总是要比从电机读取的机械限位要大一点 也就是遥控器接收到的值大一点 才能符合真的限位
    //只有pitch轴和roll轴这样
    static constexpr float pitch_max_value = 0.6f;
    static constexpr float pitch_min_value = -0.32f;

    static constexpr float roll_max_value = 0.3f;
    static constexpr float roll_min_value = -0.33f;

    static constexpr float yaw_to_roll = 0.08f;
};
}

#endif
