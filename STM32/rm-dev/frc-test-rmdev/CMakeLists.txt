# Set the minimum version of CMake that can be used
cmake_minimum_required(VERSION 3.20)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
set(CMAKE_C_COMPILER_FORCED TRUE)
set(CMAKE_CXX_COMPILER_FORCED TRUE)

# Set target definition variables required by the toolchain file
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/target.cmake)

# Set the toolchain file for cross-compilation
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/gcc-arm-none-eabi.cmake)

# Project definition
project(frc-test-rmdev)

# Enable CMake support for ASM and C/CPP languages
enable_language(C ASM)

set(PROJECT_NAME frc-test-rmdev)

# Create an executable object type
add_executable(${PROJECT_NAME})

# Include additional CMake files for flags, components, and files
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/flags.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/components.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/files.cmake)

# Add sources to executable/library
target_sources(${PROJECT_NAME} PRIVATE
  # User defined sources
  Drivers/io.c
  Drivers/uart.c
  Drivers/dbus.c
  Drivers/pwm.c
  Drivers/buzzer.c
  Drivers/imu.c
  Src/app_tasks.c
  Src/board_clock.c
  Src/stm32f4xx_it.c

  # STM32Cube USB CDC application and target glue
  USB_DEVICE/App/usb_device.c
  USB_DEVICE/App/usbd_desc.c
  USB_DEVICE/App/usbd_cdc_if.c
  USB_DEVICE/Target/usbd_conf.c

  # STM32Cube USB Device middleware
  Middlewares/USB_Device/Core/usbd_core.c
  Middlewares/USB_Device/Core/usbd_ctlreq.c
  Middlewares/USB_Device/Core/usbd_ioreq.c
  Middlewares/USB_Device/Class/CDC/usbd_cdc.c

  # CMSIS system and the Cube HAL modules needed by USB OTG FS
  Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd_ex.c
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usb.c

  # FreeRTOS kernel
  ThirdParty/FreeRTOS/tasks.c
  ThirdParty/FreeRTOS/queue.c
  ThirdParty/FreeRTOS/list.c
  ThirdParty/FreeRTOS/timers.c
  ThirdParty/FreeRTOS/event_groups.c
  ThirdParty/FreeRTOS/stream_buffer.c
  ThirdParty/FreeRTOS/croutine.c
  ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F/port.c
  ThirdParty/FreeRTOS/portable/MemMang/heap_4.c
)

# Add include directories
target_include_directories(${PROJECT_NAME} PRIVATE
  # User defined include directories
  ${CMAKE_CURRENT_SOURCE_DIR}/Drivers
  ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Include
  ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F4xx/Include
  ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc
  ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
  ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/USB_Device/Core/Inc
  ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/USB_Device/Class/CDC/Inc
  ${CMAKE_CURRENT_SOURCE_DIR}/USB_DEVICE/App
  ${CMAKE_CURRENT_SOURCE_DIR}/USB_DEVICE/Target
  ${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/FreeRTOS/include
  ${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F
)

# Compiler options
target_compile_options(${PROJECT_NAME} PRIVATE
  # User defined compiler options
)

# Add additional defined symbols
target_compile_definitions(${PROJECT_NAME} PRIVATE
  USE_HAL_DRIVER
  STM32F407xx
  HSE_VALUE=12000000U
)

# Linker options
target_link_options(${PROJECT_NAME} PRIVATE 
  # User defined linker options
)

# Add additional linker search directories
target_link_directories(${PROJECT_NAME} PRIVATE
  # Add additional linker search directories here
)

# Add additional linker libraries
target_link_libraries(${PROJECT_NAME} PRIVATE
  # Add additional linker libraries here
)
