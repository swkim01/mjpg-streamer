mjpg-streamer input plugin: input_rpifb
==========================================

MJPEG Streamer with rasberry pi framebuffer input plugin (based on mmal source code)   
Instructions
============

If you ran the basic build, you can run from the mjpeg streamer experimental
folder with:
```
export LD_LIBRARY_PATH=.
./mjpg_streamer -o "output_http.so -w ./www" -i "input_fb.so"
```

Here's some help for this input plugin:
```
---------------------------------------------------------------
Help for input plugin..: raspberry pi framebuffer input plugin
---------------------------------------------------------------
The following parameters can be passed to this plugin:

[-r | --resolution ]...: the resolution of the framebuffer device,
                         can be one of the following strings:
                         QSIF QCIF CGA QVGA CIF VGA 
                         SVGA XGA SXGA UXGA WUXGA
                         or a custom value like the following
                         example: 640x480
[-fps | --framerate]...: set video framerate, default 5 frame/sec
---------------------------------------------------------------
```
