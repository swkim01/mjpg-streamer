
find_package(X11)

if (EXISTS /opt/vc/include)
    set(HAS_RASPI ON)
else()
    set(HAS_RASPI OFF)
endif()

MJPG_STREAMER_PLUGIN_OPTION(input_fb "Framebuffer input plugin")

if (PLUGIN_INPUT_FB)
    
    if (HAS_RASPI)
      add_definitions(-DLINUX -D_GNU_SOURCE -DRASPI)
    else ()
      add_definitions(-DLINUX -D_GNU_SOURCE)
    endif (HAS_RASPI)

    if (X11_FOUND)
      add_definitions(-DX11)
    endif (X11_FOUND)
    
    if (NOT JPEG_LIB)
        add_definitions(-DNO_LIBJPEG)
    endif (NOT JPEG_LIB)

    if (HAS_RASPI)
        include_directories(/opt/vc/include)
        include_directories(/opt/vc/include/interface/vcos/pthreads)
        include_directories(/opt/vc/include/interface/vmcs_host/linux)

        link_directories(/opt/vc/lib)
    endif (HAS_RASPI)

    if (X11_FOUND)
        include_directories(${X11_INCLUDE_DIR})
    endif (X11_FOUND)

    MJPG_STREAMER_PLUGIN_COMPILE(input_fb input_fb.c jpeg_utils.c)

    if (JPEG_LIB)
      if (HAS_RASPI)
        if (X11_FOUND)
          target_link_libraries(input_fb ${JPEG_LIB} ${X11_LIBRARIES} bcm_host)
        else ()
          target_link_libraries(input_fb ${JPEG_LIB} bcm_host)
        endif (X11_FOUND)
      elseif (X11_FOUND)
          target_link_libraries(input_fb ${JPEG_LIB} ${X11_LIBRARIES})
      else ()
          target_link_libraries(input_fb ${JPEG_LIB})
      endif (HAS_RASPI)
    endif (JPEG_LIB)

endif()
