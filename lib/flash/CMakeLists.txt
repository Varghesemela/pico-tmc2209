project(flash)
add_library(${PROJECT_NAME})
# include_directories(bahar ka folder)
target_sources(${PROJECT_NAME}
    PRIVATE
        flash.cpp
        flash.h
)
target_link_libraries(${PROJECT_NAME} 
    PUBLIC 
        pico_stdlib
        pico_multicore
        hardware_flash
        hardware_sync
        hardware_irq
)