
find_package(X11)

if (EXISTS /opt/vc/include)
    set(HAS_RASPI ON)
else()
    set(HAS_RASPI OFF)
endif()

MJPG_STREAMER_PLUGIN_OPTION(input_fb "Framebuffer input plugin" ONLYIF X11_FOUND)

if (PLUGIN_INPUT_FB)
    
    if (HAS_RASPI)
      add_definitions(-DLINUX -D_GNU_SOURCE -DRASPI)
    elseif (X11_FOUND)
      add_definitions(-DLINUX -D_GNU_SOURCE -DX11)
    else ()
      add_definitions(-DLINUX -D_GNU_SOURCE)
    endif (HAS_RASPI)

    if (NOT JPEG_LIB)
        add_definitions(-DNO_LIBJPEG)
    endif (NOT JPEG_LIB)

    if (HAS_RASPI)
        include_directories(/opt/vc/include)
        include_directories(/opt/vc/include/interface/vcos/pthreads)
        include_directories(/opt/vc/include/interface/vmcs_host/linux)

        link_directories(/opt/vc/lib)
    elseif (X11_FOUND)
        include_directories(${X11_INCLUDE_DIR})
    endif (HAS_RASPI)

    MJPG_STREAMER_PLUGIN_COMPILE(input_fb input_fb.c jpeg_utils.c)

    if (JPEG_LIB)
      if (HAS_RASPI)
          target_link_libraries(input_fb ${JPEG_LIB} bcm_host)
      elseif (X11_FOUND)
          target_link_libraries(input_fb ${JPEG_LIB} ${X11_LIBRARIES})
      else ()
          target_link_libraries(input_fb ${JPEG_LIB})
      endif (HAS_RASPI)
    endif (JPEG_LIB)

endif()
