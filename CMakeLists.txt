cmake_minimum_required(VERSION 3.23)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/stm32-cmake/cmake/stm32_gcc.cmake)

get_filename_component(BASE_FOLDER ${CMAKE_CURRENT_SOURCE_DIR} NAME)
set(PROJECT_NAME ${BASE_FOLDER})
project(${PROJECT_NAME} C ASM)

find_package(CMSIS COMPONENTS STM32H743VI_M7 REQUIRED)
find_package(FreeRTOS COMPONENTS ARM_CM7 STM32H7)

set(PROJECT_SOURCES
    src/main.c
)

add_executable(${PROJECT_NAME} ${PROJECT_SOURCES})
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(${PROJECT_NAME}
    FreeRTOS::STM32::H7::M7::ARM_CM7
    FreeRTOS::STM32::H7::M7::Heap::4
    CMSIS::STM32::H743VI::M7
    STM32::Nano
    STM32::NoSys
)

stm32_print_size_of_target(${PROJECT_NAME})
stm32_generate_binary_file(${PROJECT_NAME})

add_custom_command(
    OUTPUT stm32_flashed
    DEPENDS ${PROJECT_NAME}
    COMMAND st-flash --reset write
        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.bin
        0x08000000
    VERBATIM
)
add_custom_target(flash DEPENDS stm32_flashed)

