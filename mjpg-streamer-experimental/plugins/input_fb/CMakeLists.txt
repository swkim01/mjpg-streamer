
if (EXISTS /opt/vc/include)
    set(HAS_RASPI ON)
else()
    set(HAS_RASPI OFF)
endif()

MJPG_STREAMER_PLUGIN_OPTION(input_fb "Raspberry Pi framebuffer input plugin"
                            ONLYIF HAS_RASPI)

if (PLUGIN_INPUT_FB)
    
    add_definitions(-DLINUX -D_GNU_SOURCE)
    
    if (NOT JPEG_LIB)
        add_definitions(-DNO_LIBJPEG)
    endif (NOT JPEG_LIB)

    include_directories(/opt/vc/include)
    include_directories(/opt/vc/include/interface/vcos/pthreads)
    include_directories(/opt/vc/include/interface/vmcs_host/linux)

    link_directories(/opt/vc/lib)

    MJPG_STREAMER_PLUGIN_COMPILE(input_fb input_fb.c jpeg_utils.c)

    if (JPEG_LIB)
        target_link_libraries(input_fb ${JPEG_LIB} bcm_host)
    endif (JPEG_LIB)

endif()
