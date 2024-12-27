cmake_minimum_required(VERSION 3.22)

project(stm32cubemx)
add_library(stm32cubemx INTERFACE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

target_compile_definitions(stm32cubemx INTERFACE 
	USE_HAL_DRIVER 
	STM32F042x6
    $<$<CONFIG:Debug>:DEBUG>
)

target_include_directories(stm32cubemx INTERFACE
    ../../USB_DEVICE/App
    ../../USB_DEVICE/Target
    ../../Core/Inc
    /home/nick/STM32Cube/Repository/STM32Cube_FW_F0_V1.11.5/Drivers/STM32F0xx_HAL_Driver/Inc
    /home/nick/STM32Cube/Repository/STM32Cube_FW_F0_V1.11.5/Drivers/STM32F0xx_HAL_Driver/Inc/Legacy
    /home/nick/STM32Cube/Repository/STM32Cube_FW_F0_V1.11.5/Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    /home/nick/STM32Cube/Repository/STM32Cube_FW_F0_V1.11.5/Middlewares/ST/STM32_USB_Device_Library/Class/HID/Inc
    /home/nick/STM32Cube/Repository/STM32Cube_FW_F0_V1.11.5/Drivers/CMSIS/Device/ST/STM32F0xx/Include
    /home/nick/STM32Cube/Repository/STM32Cube_FW_F0_V1.11.5/Drivers/CMSIS/Include
)

target_sources(stm32cubemx INTERFACE
    ../../Core/Src/main.c
    ../../Core/Src/stm32f0xx_it.c
    ../../Core/Src/stm32f0xx_hal_msp.c
    ../../USB_DEVICE/App/usb_device.c
    ../../USB_DEVICE/App/usbd_desc.c
    ../../USB_DEVICE/Target/usbd_conf.c
    /home/nick/STM32Cube/Repository/STM32Cube_FW_F0_V1.11.5/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_usb.c
    /home/nick/STM32Cube/Repository/STM32Cube_FW_F0_V1.11.5/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_pcd.c
    /home/nick/STM32Cube/Repository/STM32Cube_FW_F0_V1.11.5/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_pcd_ex.c
    /home/nick/STM32Cube/Repository/STM32Cube_FW_F0_V1.11.5/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_rcc.c
    /home/nick/STM32Cube/Repository/STM32Cube_FW_F0_V1.11.5/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_rcc_ex.c
    /home/nick/STM32Cube/Repository/STM32Cube_FW_F0_V1.11.5/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal.c
    /home/nick/STM32Cube/Repository/STM32Cube_FW_F0_V1.11.5/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_i2c.c
    /home/nick/STM32Cube/Repository/STM32Cube_FW_F0_V1.11.5/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_i2c_ex.c
    /home/nick/STM32Cube/Repository/STM32Cube_FW_F0_V1.11.5/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_gpio.c
    /home/nick/STM32Cube/Repository/STM32Cube_FW_F0_V1.11.5/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_dma.c
    /home/nick/STM32Cube/Repository/STM32Cube_FW_F0_V1.11.5/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_cortex.c
    /home/nick/STM32Cube/Repository/STM32Cube_FW_F0_V1.11.5/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_pwr.c
    /home/nick/STM32Cube/Repository/STM32Cube_FW_F0_V1.11.5/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_pwr_ex.c
    /home/nick/STM32Cube/Repository/STM32Cube_FW_F0_V1.11.5/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_flash.c
    /home/nick/STM32Cube/Repository/STM32Cube_FW_F0_V1.11.5/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_flash_ex.c
    /home/nick/STM32Cube/Repository/STM32Cube_FW_F0_V1.11.5/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_exti.c
    /home/nick/STM32Cube/Repository/STM32Cube_FW_F0_V1.11.5/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_spi.c
    /home/nick/STM32Cube/Repository/STM32Cube_FW_F0_V1.11.5/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_spi_ex.c
    ../../Core/Src/system_stm32f0xx.c
    /home/nick/STM32Cube/Repository/STM32Cube_FW_F0_V1.11.5/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
    /home/nick/STM32Cube/Repository/STM32Cube_FW_F0_V1.11.5/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
    /home/nick/STM32Cube/Repository/STM32Cube_FW_F0_V1.11.5/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
    /home/nick/STM32Cube/Repository/STM32Cube_FW_F0_V1.11.5/Middlewares/ST/STM32_USB_Device_Library/Class/HID/Src/usbd_hid.c
    ../../Core/Src/sysmem.c
    ../../Core/Src/syscalls.c
    ../../startup_stm32f042x6.s
)

target_link_directories(stm32cubemx INTERFACE
)

target_link_libraries(stm32cubemx INTERFACE
)

# Validate that STM32CubeMX code is compatible with C standard
if(CMAKE_C_STANDARD LESS 11)
    message(ERROR "Generated code requires C11 or higher")
endif()


