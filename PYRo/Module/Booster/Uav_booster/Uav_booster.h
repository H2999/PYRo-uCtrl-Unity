#ifndef PYRO_UAV_BOOSTER_H
#define PYRO_UAV_BOOSTER_H

#include "pyro_module_base.h"

namespace PYRo
{
struct uav_booster_cmd_t final : public pyro::cmd_base_t
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

struct uav_booster_cfg_t{};

class uav_booster_t : public pyro::module_base_t<uav_booster_t,uav_booster_cmd_t,uav_booster_cfg_t>
{
    public:
};

};

#endif // PYRO_UAV_BOOSTER_H
