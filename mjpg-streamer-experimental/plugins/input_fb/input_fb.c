/*******************************************************************************
# RaspberryPi-Framebuffer streaming input-plugin for MJPG-streamer             #
#                                                                              #
# This package work with the RaspberryPi fb with the mjpeg feature             #
#                                                                              #
# Copyright (C) 2005 2006 Laurent Pinchart &&  Michel Xhaard                   #
#                    2007 Lucas van Staden                                     #
#                    2007 Tom Stöveken                                         #
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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <getopt.h>
#include <pthread.h>
#include <syslog.h>
#include <fcntl.h>
#include <linux/fb.h>
#include <sys/mman.h>

#include <bcm_host.h>

#include "../../utils.h"
//#include "../../mjpg_streamer.h"
#include "rpifb.h"
#include "huffman.h"
#include "jpeg_utils.h"

#define INPUT_PLUGIN_NAME "RaspberryPi Framebuffer grabber"

/*
 * UVC resolutions mentioned at: (at least for some webcams)
 * http://www.quickcamteam.net/hcl/frame-format-matrix/
 */
static const struct {
    const char *string;
    const int width, height;
} resolutions[] = {
    { "QSIF", 160,  120  },
    { "QCIF", 176,  144  },
    { "CGA",  320,  200  },
    { "QVGA", 320,  240  },
    { "CIF",  352,  288  },
    { "VGA",  640,  480  },
    { "SVGA", 800,  600  },
    { "XGA",  1024, 768  },
    { "SXGA", 1280, 1024 }
};

/* private functions and variables to this plugin */
static globals *pglobal;
static int gquality = 80;
static unsigned int minimum_size = 0;

void *fb_thread(void *);
void fb_cleanup(void *);
void help(void);
int input_cmd(int plugin, unsigned int control, unsigned int group, int value);


/*** plugin interface functions ***/
/******************************************************************************
Description.: This function ializes the plugin. It parses the commandline-
              parameter and stores the default and parsed values in the
              appropriate variables.
Input Value.: param contains among others the command-line string
Return Value: 0 if everything is fine
              1 if "--help" was triggered, in this case the calling programm
              should stop running and leave.
******************************************************************************/
int input_init(input_parameter *param, int id)
{
    char *s;
    int width = 640, height = 480, fps = 30, format = VC_IMAGE_RGB888, i;
    int ret;
    struct vdIn *vd;

    /* initialize the mutes variable */
    if(pthread_mutex_init(&fbs[id].controls_mutex, NULL) != 0) {
        IPRINT("could not initialize mutex variable\n");
        exit(EXIT_FAILURE);
    }

    param->argv[0] = INPUT_PLUGIN_NAME;

    /* show all parameters for DBG purposes */
    for(i = 0; i < param->argc; i++) {
        DBG("argv[%d]=%s\n", i, param->argv[i]);
    }

    /* parse the parameters */
    reset_getopt();
    while(1) {
        int option_index = 0, c = 0;
        static struct option long_options[] = {
            {"h", no_argument, 0, 0},
            {"help", no_argument, 0, 0},
            {"r", required_argument, 0, 0},
            {"resolution", required_argument, 0, 0},
            {"f", required_argument, 0, 0},
            {"fps", required_argument, 0, 0},
            {"y", no_argument, 0, 0},
            {"yuv", no_argument, 0, 0},
            {"q", required_argument, 0, 0},
            {"quality", required_argument, 0, 0},
            {"m", required_argument, 0, 0},
            {"minimum_size", required_argument, 0, 0},
            {"n", no_argument, 0, 0},
            {0, 0, 0, 0}
        };

        /* parsing all parameters according to the list above is sufficent */
        c = getopt_long_only(param->argc, param->argv, "", long_options, &option_index);

        /* no more options to parse */
        if(c == -1) break;

        /* unrecognized option */
        if(c == '?') {
            help();
            return 1;
        }

        /* dispatch the given options */
        switch(option_index) {
            /* h, help */
        case 0:
        case 1:
            DBG("case 0,1\n");
            help();
            return 1;
            break;

            /* r, resolution */
        case 2:
        case 3:
            DBG("case 2,3\n");
            width = -1;
            height = -1;

            /* try to find the resolution in lookup table "resolutions" */
            for(i = 0; i < LENGTH_OF(resolutions); i++) {
                if(strcmp(resolutions[i].string, optarg) == 0) {
                    width  = resolutions[i].width;
                    height = resolutions[i].height;
                }
            }
            /* done if width and height were set */
            if(width != -1 && height != -1)
                break;
            /* parse value as decimal value */
            width  = strtol(optarg, &s, 10);
            height = strtol(s + 1, NULL, 10);
            printf("width, height=%d, %d\n", width, height);
            break;

            /* f, fps */
        case 4:
        case 5:
            DBG("case 4,5\n");
            fps = atoi(optarg);
            break;

            /* y, yuv */
        case 6:
        case 7:
            DBG("case 6,7\n");
            //format = VC_IMAGE_YUV420;
            break;

            /* q, quality */
        case 8:
        case 9:
            DBG("case 8,9\n");
            gquality = MIN(MAX(atoi(optarg), 0), 100);
            break;

            /* m, minimum_size */
        case 10:
        case 11:
            DBG("case 10,11\n");
            minimum_size = MAX(atoi(optarg), 0);
            break;

        default:
            DBG("default case\n");
            help();
            return 1;
        }
    }
    DBG("input id: %d\n", id);
    fbs[id].id = id;
    fbs[id].pglobal = param->global;

    /* allocate fb datastructure */
    fbs[id].videoIn = malloc(sizeof(struct vdIn));
    if(fbs[id].videoIn == NULL) {
        IPRINT("not enough memory for videoIn\n");
        exit(EXIT_FAILURE);
    }
    memset(fbs[id].videoIn, 0, sizeof(struct vdIn));

    /* display the parsed values */
    IPRINT("Using RaspberryPi Framebuffer device.:\n");
    IPRINT("Desired Resolution: %i x %i\n", width, height);
    IPRINT("Frames Per Second.: %i\n", fps);
    IPRINT("Format............: %s\n", (format == VC_IMAGE_YUV420) ? "YUV" : "RGB");
    IPRINT("JPEG Quality......: %d\n", gquality);

    DBG("vdIn pn: %d\n", id);

    /* open video device and prepare data structure */
    bcm_host_init();

    vd = fbs[id].videoIn;
    vd->display = vc_dispmanx_display_open(0);
    if (!vd->display) {
        IPRINT("Unable to open primary display");
        closelog();
        exit(EXIT_FAILURE);
    }
    ret = vc_dispmanx_display_get_info(vd->display, &vd->display_info);
    if (ret) {
        IPRINT("Unable to get primary display information");
        closelog();
        exit(EXIT_FAILURE);
    }
    IPRINT("Primary display is %d x %d\n", vd->display_info.width, vd->display_info.height);
    vd->width = width;
    vd->height = height;
    vd->fps = fps;
    vd->formatIn = format;
    vd->screen_resource = vc_dispmanx_resource_create(format, width, height, &vd->image_prt);
    if (!vd->screen_resource) {
        IPRINT("Unable to create screen buffer");
        vc_dispmanx_display_close(vd->display);
        closelog();
        exit(EXIT_FAILURE);
    }
    vd->framesizeIn = (vd->width * vd->height << 1);
    switch(vd->formatIn) {
    case VC_IMAGE_RGB565:
        vd->bits_per_pixel = 16;
        vd->framebuffer =
            (unsigned char *) calloc(1, (size_t) vd->width * (vd->height + 8) * 2);
        break;
    case VC_IMAGE_RGB888:
        vd->bits_per_pixel = 24;
        vd->framebuffer =
            (unsigned char *) calloc(1, (size_t) vd->width * (vd->height + 8) * 3);
        break;
    case VC_IMAGE_YUV420:
        vd->bits_per_pixel = 16;
        vd->framebuffer =
            (unsigned char *) calloc(1, (size_t) vd->framesizeIn);
        break;
    default:
        fprintf(stderr, " should never arrive exit fatal !!\n");
        goto error;
        break;
    }

    vc_dispmanx_rect_set(&vd->rect1, 0, 0, width, height);

error:
    return 0;
}

/******************************************************************************
Description.: Stops the execution of worker thread
Input Value.: -
Return Value: always 0
******************************************************************************/
int input_stop(int id)
{
    DBG("will cancel fbera thread #%02d\n", id);
    pthread_cancel(fbs[id].threadID);
    return 0;
}

/******************************************************************************
Description.: spins of a worker thread
Input Value.: -
Return Value: always 0
******************************************************************************/
int input_run(int id)
{
    fbs[id].pglobal->in[id].buf = malloc(fbs[id].videoIn->framesizeIn);
    if(fbs[id].pglobal->in[id].buf == NULL) {
        fprintf(stderr, "could not allocate memory\n");
        exit(EXIT_FAILURE);
    }

    DBG("launching fb thread #%02d\n", id);
    /* create thread and pass context to thread function */
    pthread_create(&(fbs[id].threadID), NULL, fb_thread, &(fbs[id]));
    pthread_detach(fbs[id].threadID);
    return 0;
}

/*** private functions for this plugin below ***/
/******************************************************************************
Description.: print a help message to stderr
Input Value.: -
Return Value: -
******************************************************************************/
void help(void)
{
    int i;

    fprintf(stderr, " ---------------------------------------------------------------\n" \
    " Help for input plugin..: "INPUT_PLUGIN_NAME"\n" \
    " ---------------------------------------------------------------\n" \
    " The following parameters can be passed to this plugin:\n\n" \
    " [-r | --resolution ]...: the resolution of the video device,\n" \
    "                          can be one of the following strings:\n" \
    "                          ");

    for(i = 0; i < LENGTH_OF(resolutions); i++) {
        fprintf(stderr, "%s ", resolutions[i].string);
        if((i + 1) % 6 == 0)
            fprintf(stderr, "\n                          ");
    }
    fprintf(stderr, "\n                          or a custom value like the following" \
    "\n                          example: 640x480\n");

    fprintf(stderr, " [-f | --fps ]..........: frames per second\n" \
    " [-y | --yuv ]..........: enable YUYV format and disable MJPEG mode\n" \
    " [-q | --quality ]......: JPEG compression quality in percent \n" \
    "                          (activates YUYV format, disables MJPEG)\n" \
    " [-m | --minimum_size ].: drop frames smaller then this limit, useful\n" \
    "                          if the webfb produces small-sized garbage frames\n" \
    "                          may happen under low light conditions\n" \
    " ---------------------------------------------------------------\n\n");
}

/******************************************************************************
Description.: this thread worker grabs a frame and copies it to the global buffer
Input Value.: unused
Return Value: unused, always NULL
******************************************************************************/
void *fb_thread(void *arg)
{
    struct vdIn *vd;
    context *pcontext = arg;
    pglobal = pcontext->pglobal;

    /* set cleanup handler to cleanup allocated ressources */
    pthread_cleanup_push(fb_cleanup, pcontext);

    vd = pcontext->videoIn;
    while(!pglobal->stop) {
        while(vd->streamingState == STREAMING_PAUSED) {
            usleep(1); // maybe not the best way so FIXME
        }

        /* grab a frame */
        vc_dispmanx_snapshot(vd->display, vd->screen_resource, 0);
        vc_dispmanx_resource_read_data(vd->screen_resource, &vd->rect1, vd->framebuffer, vd->width * vd->bits_per_pixel / 8);

        DBG("received frame of size: %d from plugin: %d\n", pcontext->videoIn->buf.bytesused, pcontext->id);

        /* copy JPG picture to global buffer */
        pthread_mutex_lock(&pglobal->in[pcontext->id].db);

        /*
         * If capturing in YUV mode convert to JPEG now.
         * This compression requires many CPU cycles, so try to avoid YUV or
         * RGB format. Getting JPEGs straight from the fb, is one of the
         * major advantages of Linux compatible devices.
         */
        if(vd->formatIn == VC_IMAGE_YUV420) {
            DBG("compressing yuv420 frame from input: %d\n", (int)pcontext->id);
            pglobal->in[pcontext->id].size = compress_yuv420_to_jpeg(vd, pglobal->in[pcontext->id].buf, vd->framesizeIn, gquality);
        } else {
            DBG("compressing rgb frame from input: %d\n", (int)pcontext->id);
            pglobal->in[pcontext->id].size = compress_rgb888_to_jpeg(vd, pglobal->in[pcontext->id].buf, vd->framesizeIn, gquality);
        }

#if 0
        /* motion detection can be done just by comparing the picture size, but it is not very accurate!! */
        if((prev_size - global->size)*(prev_size - global->size) > 4 * 1024 * 1024) {
            DBG("motion detected (delta: %d kB)\n", (prev_size - global->size) / 1024);
        }
        prev_size = global->size;
#endif

        /* signal fresh_frame */
        pthread_cond_broadcast(&pglobal->in[pcontext->id].db_update);
        pthread_mutex_unlock(&pglobal->in[pcontext->id].db);

        /* only use usleep if the fps is below 5, otherwise the overhead is too long */
        if(vd->fps < 5) {
            DBG("waiting for next frame for %d us\n", 1000 * 1000 / vd->fps);
            usleep(1000 * 1000 / vd->fps);
        } else {
            DBG("waiting for next frame\n");
        }
    }

    DBG("leaving input thread, calling cleanup function now\n");
    pthread_cleanup_pop(1);

    return NULL;
}

/******************************************************************************
Description.:
Input Value.:
Return Value:
******************************************************************************/
void fb_cleanup(void *arg)
{
    static unsigned char first_run = 1;
    context *pcontext = arg;
    pglobal = pcontext->pglobal;
    if(!first_run) {
        DBG("already cleaned up ressources\n");
        return;
    }

    first_run = 0;
    IPRINT("cleaning up ressources allocated by input thread\n");

    if(pcontext->videoIn->framebuffer != NULL)
        free(pcontext->videoIn->framebuffer);
    //if(pcontext->videoIn->screen_resource != NULL)
    vc_dispmanx_resource_delete(pcontext->videoIn->screen_resource);
    //if(pcontext->videoIn->display != NULL)
    vc_dispmanx_display_close(pcontext->videoIn->display);
    if(pcontext->videoIn != NULL) free(pcontext->videoIn);
    if(pglobal->in[pcontext->id].buf != NULL)
        free(pglobal->in[pcontext->id].buf);
}

/******************************************************************************
Description.: process commands
Input Value.:
Return Value:
******************************************************************************/
int input_cmd(int plugin_number, unsigned int control_id, unsigned int group, int value)
{
    int ret = -1;
    DBG("Requested cmd (id: %d) for the %d plugin. Group: %d value: %d\n", control_id, plugin_number, group, value);
    return ret;
}

