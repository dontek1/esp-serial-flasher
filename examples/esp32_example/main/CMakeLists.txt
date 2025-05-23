#set(SINGLE_TARGET_SUPPORT 1)

set(srcs main.c ../../common/example_common.c)
set(include_dirs . ../../common)

if(DEFINED SINGLE_TARGET_SUPPORT)
    message(STATUS "SINGLE_TARGET_SUPPORT defined")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DSINGLE_TARGET_SUPPORT")

    #set(ENABLE_HTTP_DOWNLOAD 1)
    if(DEFINED ENABLE_HTTP_DOWNLOAD)
        message(STATUS "ENABLE_HTTP_DOWNLOAD defined")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DENABLE_HTTP_DOWNLOAD")
        
        idf_component_register(SRCS ${srcs}
                                INCLUDE_DIRS ${include_dirs})
    else()
        message(STATUS "ENABLE_HTTP_DOWNLOAD NOT defined")       
        set(bin_dir ../../../../https_mbedtls/build)
        
        idf_component_register(SRCS ${srcs}
                                INCLUDE_DIRS ${include_dirs}
                                EMBED_FILES "${bin_dir}/bootloader/bootloader.bin"
                                            "${bin_dir}/partition_table/partition-table.bin"
                                            "${bin_dir}/https_mbedtls.bin")
    endif()
else()
    message(STATUS "SINGLE_TARGET_SUPPORT NOT defined")

    idf_component_register(SRCS ${srcs}
                            INCLUDE_DIRS ${include_dirs})
    set(target ${COMPONENT_LIB})

    # Embed binaries into the app.
    # In ESP-IDF this can also be done using EMBED_FILES option of idf_component_register.
    # Here an external tool is used to make file embedding similar with other ports.
    include(${CMAKE_CURRENT_LIST_DIR}/../../common/bin2array.cmake)
    create_resources(${CMAKE_CURRENT_LIST_DIR}/../../binaries/Hello-world ${CMAKE_BINARY_DIR}/binaries.c)
    set_property(SOURCE ${CMAKE_BINARY_DIR}/binaries.c PROPERTY GENERATED 1)
    target_sources(${target} PRIVATE ${CMAKE_BINARY_DIR}/binaries.c)   
endif()

