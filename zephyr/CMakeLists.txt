if(CONFIG_KYVERNITIS)
    set(KYVERNITIS_DIR ${ZEPHYR_CURRENT_MODULE_DIR})
    zephyr_include_directories(${KYVERNITIS_DIR}/include)
    zephyr_library()
    zephyr_library_sources(
        ${KYVERNITIS_DIR}/lib/kyvernitis.c
        ${KYVERNITIS_DIR}/lib/drive.c
        )
endif()
