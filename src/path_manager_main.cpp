/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Example User <mail@example.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file path_manager_main.cpp
 * Path manager main function for autopilot path manager in chapter 11 of UAVbook, see http://uavbook.byu.edu/doku.php
 * Author: @author Gary Ellingson <gary.ellingson@byu.edu>
 */

#include <nuttx/config.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <fcntl.h>
#include <nuttx/sched.h>
#include <sys/prctl.h>
#include <termios.h>
#include <errno.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <systemlib/param/param.h>
#include <systemlib/perf_counter.h>

#include <lib/mathlib/mathlib.h>

#include "path_manager_base.h"
#include "path_manager_example.h"


extern "C" __EXPORT int path_manager_main(int argc, char *argv[]);

static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */

//funtion prototypes
int path_manager_thread_main(int argc, char *argv[]);
static void usage(const char *reason);

static void usage(const char *reason)
{
    if (reason) {
        fprintf(stderr, "%s\n", reason);
    }

    fprintf(stderr, "usage: path_manager {start|stop|status} [-p <additional params>]\n\n");
    exit(1);
}


/**
 * The path_manager app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_spawn_cmd().
 */

int path_manager_main(int argc, char *argv[])
{
    if (argc < 1) {
        usage("missing command");
    }

    if (!strcmp(argv[1], "start")) {

        if (thread_running) {
            printf("path_manager already running\n");
            /* this is not an error */
            exit(0);
        }

        thread_should_exit = false;
        daemon_task = task_spawn_cmd("path_manager",
                         SCHED_DEFAULT,
                         SCHED_PRIORITY_DEFAULT,
                         2000,
                         path_manager_thread_main,
                         (argv) ? (char * const *)&argv[2] : (char * const *)NULL);
        exit(0);
    }

    if (!strcmp(argv[1], "stop")) {
        thread_should_exit = true;
        exit(0);
    }

    if (!strcmp(argv[1], "status")) {
        if (thread_running) {
            warnx("running");
            exit(0);

        } else {
            warnx("not started");
            exit(1);
        }

        exit(0);
    }

    usage("unrecognized command");
    exit(1);

	return OK;
}

int path_manager_thread_main(int argc, char *argv[])
{
    warnx("[daemon] starting\n");

    thread_running = true;

    path_manager_base* pm = new path_manager_example();
    while(!thread_should_exit)
    {
        pm->spin();
    }

    warnx("[daemon] exiting.\n");
    thread_running = false;
    return 0;
}
