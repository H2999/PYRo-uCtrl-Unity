target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        PYRo/Application/Mission/Uav/pyro_uav_gimbal_app.cpp
        PYRo/Application/Mission/Uav/pyro_uav_booster_app.cpp

        PYRo/Module/Gimbal/Uav/pyro_uav_gimbal.cpp
        PYRo/Module/Gimbal/Uav/fsm/pyro_active_state.cpp
        PYRo/Module/Gimbal/Uav/fsm/pyro_passive_state.cpp

        PYRo/Module/Booster/Uav_booster/fsm/pyro_shoot_active_state.cpp
#        PYRo/Module/Booster/Uav_booster/fsm/pyro_shoot_busy_state.cpp
#        PYRo/Module/Booster/Uav_booster/fsm/pyro_shoot_homing_state.cpp
        PYRo/Module/Booster/Uav_booster/fsm/pyro_shoot_passive_state.cpp
#        PYRo/Module/Booster/Uav_booster/fsm/pyro_shoot_ready_state.cpp
        PYRo/Module/Booster/Uav_booster/Uav_booster.cpp
)
target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC
        PYRo/Module/Gimbal/Uav
        PYRo/Module/Booster
)