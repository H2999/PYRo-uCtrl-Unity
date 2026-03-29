#include "cmsis_os.h"
#include "pyro_core_def.h"

namespace pyro
{
extern "C"
{
    extern void pyro_init_thread(void *argument);
    extern void start_debug_task(void *arg);
    extern void uav_pc_com_init(void *argument);
    extern void rc_demo(void *argument);
    status_t pyro_init_ret;
    status_t pyro_booster_init;
#if (ROBOT_ID == HERO_ID) || (ROBOT_ID == SUB_HERO_ID)
#if BOARD_ID == GIMBAL_ID
    extern void hero_gimbal_init(void *argument);
    extern void hero_booster_init(void *argument);
#elif BOARD_ID == CHASSIS_ID
    extern void hero_chassis_init(void *argument);
#endif
#elif ROBOT_ID == SENTRY_ID
#if BOARD_ID == GIMBAL_ID
    extern void sentry_gimbal_init(void *argument);
    extern void sentry_booster_init(void *argument);
#elif BOARD_ID == CHASSIS_ID
    extern void sentry_chassis_init(void *argument);
#endif
#endif
#if ROBOT_ID == INFANTRY2_ID
    extern status_t infantry2_chassis_init(void *argument);
#endif
#if ROBOT_ID == UAV_ID
    extern status_t uav_gimbal_init(void *argument);
    extern status_t uav_booster_init(void *argument);
#endif
    void start_mission_planer_task(void const *argument)
    {
        pyro_init_thread(nullptr);
#if (ROBOT_ID == HERO_ID) || (ROBOT_ID == SUB_HERO_ID)
#if BOARD_ID == GIMBAL_ID
        xTaskCreate(hero_gimbal_init, "pyro_gimbal_init", 512, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);
        xTaskCreate(hero_booster_init, "pyro_booster_init", 512, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);
#elif BOARD_ID == CHASSIS_ID
        xTaskCreate(hero_chassis_init, "pyro_chassis_init", 512, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);
#endif
#elif ROBOT_ID == SENTRY_ID
#if BOARD_ID == GIMBAL_ID
        xTaskCreate(sentry_gimbal_init, "pyro_sentry_gimbal_init", 512, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);
        xTaskCreate(sentry_booster_init, "pyro_sentry_booster_init", 512, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);
#elif BOARD_ID == CHASSIS_ID
        xTaskCreate(sentry_chassis_init, "pyro_sentry_chassis_init", 512, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);
#endif
#endif

#if ROBOT_ID == INFANTRY2_ID
        pyro_init_ret = infantry2_chassis_init(nullptr);
#endif

#if ROBOT_ID == UAV_ID
        // pyro_init_ret = uav_gimbal_init(nullptr);
        // pyro_booster_init = uav_booster_init(nullptr);
        // xTaskCreate(uav_pc_com_init, "pyro_uav_autoaim_init", 512, nullptr,
        //             configMAX_PRIORITIES - 1, nullptr);
        xTaskCreate(rc_demo,"rc_demo",256,nullptr,
                    configMAX_PRIORITIES - 1,nullptr);
#endif

#if DEBUG_MODE
        xTaskCreate(start_debug_task, "start_debug_task", 128, nullptr,
                    configMAX_PRIORITIES - 2, nullptr);
#endif

        osThreadTerminate(nullptr);
        // vTaskDelete(nullptr);
    }
}
}
