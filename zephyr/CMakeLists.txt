cmake_minimum_required(VERSION 3.5)

if (CONFIG_ESP_SERIAL_FLASHER)
    zephyr_include_directories(
        "${ZEPHYR_CURRENT_MODULE_DIR}/include"
        "${ZEPHYR_CURRENT_MODULE_DIR}/port"
        "${ZEPHYR_CURRENT_MODULE_DIR}/private_include"
    )

    zephyr_interface_library_named(esp_flasher)

    zephyr_library()

    zephyr_library_sources(${ZEPHYR_CURRENT_MODULE_DIR}/src/esp_loader.c
                ${ZEPHYR_CURRENT_MODULE_DIR}/src/esp_targets.c
                ${ZEPHYR_CURRENT_MODULE_DIR}/src/esp_stubs.c
                ${ZEPHYR_CURRENT_MODULE_DIR}/src/protocol_serial.c
                ${ZEPHYR_CURRENT_MODULE_DIR}/src/protocol_uart.c
                ${ZEPHYR_CURRENT_MODULE_DIR}/src/slip.c
                ${ZEPHYR_CURRENT_MODULE_DIR}/src/md5_hash.c
                ${ZEPHYR_CURRENT_MODULE_DIR}/port/zephyr_port.c
    )

    target_compile_definitions(esp_flasher INTERFACE SERIAL_FLASHER_INTERFACE_UART)

    zephyr_library_link_libraries(esp_flasher)

    if(DEFINED MD5_ENABLED OR CONFIG_SERIAL_FLASHER_MD5_ENABLED)
        target_compile_definitions(esp_flasher INTERFACE -DMD5_ENABLED=1)
    endif()

    if(DEFINED SERIAL_FLASHER_DEBUG_TRACE OR CONFIG_SERIAL_FLASHER_DEBUG_TRACE)
        target_compile_definitions(esp_flasher INTERFACE SERIAL_FLASHER_DEBUG_TRACE=1)
    endif()

    target_compile_definitions(esp_flasher
    INTERFACE
        SERIAL_FLASHER_WRITE_BLOCK_RETRIES=${CONFIG_SERIAL_FLASHER_WRITE_BLOCK_RETRIES}
    )

    if((DEFINED SERIAL_FLASHER_RESET_INVERT AND SERIAL_FLASHER_RESET_INVERT) OR CONFIG_SERIAL_FLASHER_RESET_INVERT)
        target_compile_definitions(esp_flasher INTERFACE SERIAL_FLASHER_RESET_INVERT=1)
    else()
        target_compile_definitions(esp_flasher INTERFACE SERIAL_FLASHER_RESET_INVERT=0)
    endif()

    if((DEFINED SERIAL_FLASHER_BOOT_INVERT AND SERIAL_FLASHER_BOOT_INVERT) OR CONFIG_SERIAL_FLASHER_BOOT_INVERT)
        target_compile_definitions(esp_flasher INTERFACE SERIAL_FLASHER_BOOT_INVERT=1)
    else()
        target_compile_definitions(esp_flasher INTERFACE SERIAL_FLASHER_BOOT_INVERT=0)
    endif()

endif()
