/*******************************************************************************
# Framebuffer streaming input-plugin for MJPG-streamer                         #
#                                                                              #
# This package work with the linux(inc. RaspberryPi) fb with the mjpeg feature #
#                                                                              #
# Copyright (C) 2005 2006 Laurent Pinchart &&  Michel Xhaard                   #
#                    2007 Lucas van Staden                                     #
#                    2007 Tom Stöveken                                         #
#                    2016 Seong-Woo Kim                                        #
#                                                                              #
# This program is free software; you can redistribute it and/or modify         #
# it under the terms of the GNU General Public License as published by         #
# the Free Software Foundation; either version 2 of the License, or            #
# (at your option) any later version.                                          #
#                                                                              #
# This program is distributed in the hope that it will be useful,              #
# but WITHOUT ANY WARRANTY; without even the implied warranty of               #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                #
# GNU General Public License for more details.                                 #
#                                                                              #
# You should have received a copy of the GNU General Public License            #
# along with this program; if not, write to the Free Software                  #
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA    #
#                                                                              #
*******************************************************************************/

#ifndef INPUT_FB_H
#define INPUT_FB_H

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#ifdef RASPI
#include <bcm_host.h>
#else
#ifdef X11
#include <X11/Xlib.h>
#include <X11/X.h>
#endif
#endif
#include <linux/fb.h>
#include "../../mjpg_streamer.h"
#define NB_BUFFER 4

#define IOCTL_RETRY 4

typedef enum _streaming_state streaming_state;
enum _streaming_state {
    STREAMING_OFF = 0,
    STREAMING_ON = 1,
    STREAMING_PAUSED = 2,
};

typedef enum _device_t device_t;
enum _device_t {
    DEVICE_XWINDOW,
    DEVICE_FB,
    DEVICE_RASPI
};

struct vdIn {
    device_t device;
#if RASPI
    DISPMANX_DISPLAY_HANDLE_T display;
    DISPMANX_MODEINFO_T display_info;
    DISPMANX_RESOURCE_HANDLE_T screen_resource;
    uint32_t image_prt;
    VC_RECT_T rect1;
#else
#ifdef X11
    Display *display;
    Window root;
    XWindowAttributes gwa;
#endif
#endif
    int fbfd;
    struct fb_var_screeninfo display_info;
    int bytes_per_pixel;
    int fbsize;
    unsigned char *buffer;

    unsigned char *framebuffer;
    streaming_state streamingState;
    int width;
    int height;
    int bits_per_pixel;
    int fps;
    int formatIn;
    int formatOut;
    int framesizeIn;
};

/* context of each camera thread */
typedef struct {
    int id;
    globals *pglobal;
    pthread_t threadID;
    pthread_mutex_t controls_mutex;
    struct vdIn *videoIn;
} context;

context fbs[MAX_INPUT_PLUGINS];

#endif
