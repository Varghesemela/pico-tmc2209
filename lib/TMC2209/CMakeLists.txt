project(TMC2209)
add_library(${PROJECT_NAME})

target_sources(${PROJECT_NAME}
    PRIVATE
        tmc2209.cpp
        tmc2209.h        
)

target_link_libraries(${PROJECT_NAME} 
    PRIVATE 
        pico_stdlib
        hardware_uart
        hardware_pwm
)