/* 
 *   Copyright (c) 2017 David Palma.
 *   All Rights Reserved.
 * 
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *   <david.palma(at)ntnu(dot)no>
 */

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/poll.h>
#include <fcntl.h>

#include <stdarg.h>
#include <termios.h>
#include <signal.h>

#ifndef OWL_DEV		
	#define NUM_FDS 1
#else
	#define NUM_FDS 2
#endif

#define OWL_GEN_HEADER 5
/* 
 * sixlo max size is 127 + 1 byte for length
 * NGHAM SPP max size is:
 *	general header: start, crc (2), type, length
 *	rx: time (4), noise, rssi, errors, flags, data (max 220)
 *	total = 233
 */
#define MAX_BUF	240

#define frame_length(frsup) ((frsup)->curr_buf_ptr - (frsup)->buffer)


const char* PATHS[] = { "/dev/char6lo", "/dev/ttyUSB0" };
const int FLAGS[] = {
		O_RDWR | O_NONBLOCK,
		O_RDWR | O_NONBLOCK | O_SYNC | O_NOCTTY
	};

struct frame_support {
	int missing_bytes;
	unsigned char *curr_buf_ptr;
	unsigned char buffer[MAX_BUF];
};

void print_frame(const unsigned char *, int);
int read_sixlo_frame(int, struct frame_support *);
int write_sixlo_frame(int, const struct frame_support *);
#ifdef OWL_DEV
int read_ngham_frame(int, struct frame_support *);
int write_ngham_frame(int, const struct frame_support *);

static int configure_owl_serial(int);
#endif

/* create array of functions for each FD */
int (*read_func[NUM_FDS]) (int fd, struct frame_support *fr);
int (*write_func[NUM_FDS]) (int fd, const struct frame_support *fr);

static sigset_t init_signal_handler();
static void set_escape_loop_flag(int);

struct NGHAMParser *parser;
static sig_atomic_t s_signal_caught = 0;

static void debug_printf(const char *, ...);
