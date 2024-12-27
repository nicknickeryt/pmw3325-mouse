function(target_stlink_flash)
    add_custom_target(flash
            openocd 
            -f ${CMAKE_SOURCE_DIR}/Flash/openocd.cfg 
            -c "program ${CMAKE_SOURCE_DIR}/build/${CMAKE_BUILD_TYPE}/${CMAKE_PROJECT_NAME}.elf verify reset exit"            
            DEPENDS ${CMAKE_PROJECT_NAME}
            COMMENT "Flashing ${CMAKE_PROJECT_NAME}.elf via ST-Link interface"
            WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
endfunction(target_stlink_flash)