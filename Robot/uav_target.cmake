target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        Robot/Application/Uav/pyro_uav_gimbal_app.cpp
        Robot/Application/Uav/pyro_uav_booster_app.cpp
        Robot/Application/Uav/pyro_pc_com.cpp

        Robot/Uav/Gimbal/pyro_uav_gimbal.cpp
        Robot/Uav/Gimbal/fsm/pyro_active_state.cpp
        Robot/Uav/Gimbal/fsm/pyro_passive_state.cpp
        Robot/Uav/Gimbal/fsm/pyro_gimbal_auto_state.cpp
        Robot/Uav/Gimbal/fsm/pyro_gimbal_rc_state.cpp

        Robot/Uav/Booster/fsm/pyro_booster_active_state.cpp
        Robot/Uav/Booster/fsm/pyro_booster_passive_state.cpp
        Robot/Uav/Booster/fsm/pyro_booster_interim_state.cpp
        Robot/Uav/Booster/fsm/pyro_booster_single_fire_state.cpp
        Robot/Uav/Booster/fsm/pyro_booster_continue_fire_state.cpp
        Robot/Uav/Booster/fsm/pyro_booster_stall_state.cpp
        Robot/Uav/Booster/fsm/pyro_auto_aim_state.cpp
        Robot/Uav/Booster/pyro_uav_booster.cpp

        Robot/Uav/auto_aim_serial/pyro_uart_comm.cpp
        Robot/Application/auto_aim/pc_info.cpp
        Robot/Uav/rc_demo.cpp
        Robot/rc/rc.cpp
)
target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC
        Robot/Uav/Booster
        Robot/Uav/Gimbal
        Robot/Uav/auto_aim_serial
        Robot/Application/auto_aim
        Robot/rc
)