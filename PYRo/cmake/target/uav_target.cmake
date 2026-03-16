target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        PYRo/Application/Mission/Uav/pyro_uav_gimbal_app.cpp
        PYRo/Application/Mission/Uav/pyro_uav_booster_app.cpp
        PYRo/Application/Mission/Uav/pyro_pc_com.cpp

        PYRo/Module/Gimbal/Uav/pyro_uav_gimbal.cpp
        PYRo/Module/Gimbal/Uav/fsm/pyro_active_state.cpp
        PYRo/Module/Gimbal/Uav/fsm/pyro_passive_state.cpp
        PYRo/Module/Gimbal/Uav/fsm/pyro_gimbal_auto_state.cpp
        PYRo/Module/Gimbal/Uav/fsm/pyro_gimbal_rc_state.cpp

        PYRo/Module/Booster/Uav_booster/fsm/pyro_booster_active_state.cpp
        PYRo/Module/Booster/Uav_booster/fsm/pyro_booster_passive_state.cpp
        PYRo/Module/Booster/Uav_booster/fsm/pyro_booster_interim_state.cpp
        PYRo/Module/Booster/Uav_booster/fsm/pyro_booster_single_fire_state.cpp
        PYRo/Module/Booster/Uav_booster/fsm/pyro_booster_continue_fire_state.cpp
        PYRo/Module/Booster/Uav_booster/fsm/pyro_booster_stall_state.cpp
        PYRo/Module/Booster/Uav_booster/fsm/pyro_auto_aim_state.cpp
        PYRo/Module/Booster/Uav_booster/pyro_uav_booster.cpp

        PYRo/signal/pyro_uart_comm.cpp
)
target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC
        PYRo/Module/Gimbal/Uav
        PYRo/Module/Booster/Uav_booster
        PYRo/signal
)