#include "pyro_dwt_drv.h"
#include "pyro_uav_booster.h"

using namespace pyro;
void uav_booster_t::fsm_active_t::state_backoff_t::enter(uav_booster_t *owner)
{
    backoff_turnback_start_time = dwt_drv_t::get_timeline_ms();
}

void uav_booster_t::fsm_active_t::state_backoff_t::execute(uav_booster_t *owner)
{
    const float turnback_time =
    dwt_drv_t::get_timeline_ms() - backoff_turnback_start_time;
    if (turnback_time > 1000.0f)
    {
        request_switch(&owner->active_state.interim_state);
        return;
    }
    owner->booster_ctx.data_ctx.target_trigger_radps = -6.0f;
    owner->trigger_speed_control();
    owner->send_trigger_command();
}

void uav_booster_t::fsm_active_t::state_backoff_t::exit(uav_booster_t *owner)
{
    owner->booster_ctx.data_ctx.target_trigger_rad = owner->booster_ctx.data_ctx.current_trigger_rad;
    owner->booster_ctx.data_ctx.target_trigger_radps = 0.0f;
}
