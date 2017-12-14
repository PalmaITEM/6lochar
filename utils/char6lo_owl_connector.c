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

#include "6loch_conn.h"
#include "ngham.h"

/* 
 * Polls the specified file descriptors and calls the corresponding
 * read or write function
 */
int main()
{
	struct pollfd pfd[NUM_FDS];
	struct frame_support frame[NUM_FDS];
	int n, curr_fd, flen;

	/* 
	 * init array of read and write functions
	 * NOTE! inverted indexes
	 */
	read_func[0] = read_sixlo_frame;
	write_func[1] = write_sixlo_frame;
#ifdef OWL_DEV
	parser = ngham_parserCreate();

	read_func[1] = read_ngham_frame;
	write_func[0] = write_ngham_frame;
#endif

	/* signal handlers */
	init_signal_handler();

	for (curr_fd=0; curr_fd < NUM_FDS; curr_fd++) {
		/* init frame support struct */
		frame[curr_fd].missing_bytes = 0;
		frame[curr_fd].curr_buf_ptr = frame[curr_fd].buffer;

		/* open file descriptors */
		pfd[curr_fd].fd = open(PATHS[curr_fd], FLAGS[curr_fd]);
		if (pfd[curr_fd].fd < 0) {
			fprintf(stderr, "Failed to open %s: %s\n",
					PATHS[curr_fd], strerror(errno));
			exit(EXIT_FAILURE);
		}
		pfd[curr_fd].events = POLLIN;
	}
#ifdef OWL_DEV
	if (configure_owl_serial(pfd[1].fd)) {
		fprintf(stderr, "Failed to configure serial port: %s\n", strerror(errno));
		/* TODO cleanup */
		exit(EXIT_FAILURE);
	}
#endif

	while (!s_signal_caught) {
		n = poll(pfd, NUM_FDS, -1); /* no timeout */
		if (n < 0) {
			fprintf(stderr, "Error in poll: %s\n", strerror(errno));
			if(errno == EAGAIN)
				continue;
			break;
		} else if (n == 0) {
			/* timeout, not used */
			fprintf(stderr, "Timeout! Should never happen!\n");
			continue;
		}
		for (curr_fd=0; curr_fd < NUM_FDS && !s_signal_caught; curr_fd++) {
			if (pfd[curr_fd].revents & POLLIN) {
				/* Needed? pfd[curr_fd].revents = 0; */

				/* 
				 * read frame corresponding to the correct fd
				 *
				 * pfd[0] --> read_sixlo_frame
				 * pfd[1] --> read_ngham_frame
				 */
				n = (*read_func[curr_fd])
					(pfd[curr_fd].fd, &frame[curr_fd]);

				if (n < 0) {
					fprintf(stderr,
						"Failed to read 6lo frame from %s\n",
							PATHS[curr_fd]);
					continue;
                                }

				if (frame[curr_fd].missing_bytes == 0) {
					/* immediately write to the other FD */
					flen = frame_length(&frame[curr_fd]);
                                        if (flen == 0)
                                                /* move along, nothing to see here... */
                                                continue;

					debug_printf("Frame complete from dev: %s! len: %d",
							PATHS[curr_fd], flen);
					print_frame(frame[curr_fd].buffer, flen);

					/* 
					 * write frame corresp. to the correct fd
					 *
					 * pfd[0] --> write_ngham_frame to pfd[1]
					 * pfd[1] --> write_sixlo_frame to pfd[0]
					 */
					if (NUM_FDS > 1)
						n = (*write_func[curr_fd]) 
							(pfd[!curr_fd].fd, 
							&frame[curr_fd]);
					/* 
					 * assuming buffered writes so
					 * not checking if all was written
					 * TODO maybe create a fifo for received
					 * packets and create a poll for writes
					*/
				}
			}
		}
	}

	if (s_signal_caught) {
		const char *signalName = 
			s_signal_caught == SIGINT ? "SIGINT" : 
			s_signal_caught == SIGTERM ? "SIGTERM" : 
			s_signal_caught == SIGHUP ? "SIGHUP" : "??";

		fprintf(stderr, "Caught signal %s\n", signalName);
	}

	for (curr_fd=0; curr_fd < NUM_FDS; curr_fd++) {
		/* close file descriptors */
		close(pfd[curr_fd].fd);
	}

#ifdef OWL_DEV
	ngham_parserDestroy(parser);
#endif
	return 0;
}

/*
 * Prints a char array/frame in hex
 */
void print_frame(const unsigned char *frame, int flen)
{
	int b;

	for (b = 0; b < flen; b++)
	{
	    if (b % 8 == 0) debug_printf(" ");
	    if (b % 16 == 0) debug_printf("\n");
	    debug_printf("%02X ", frame[b]);
	}
	debug_printf("\n");

	return;
}

/*
 * Attempts to read a 6lochar frame using the given file descriptor
 * (frame format: len byte, frame of size len)
 * @fd: file descriptor
 * @fr: support frame with buffer, current buf position and number of missing bytes
 *
 * returns:
 * < 0 if an error occurs
 * 0 otherwise (the frame may still be incomplete)
 *
 */
int read_sixlo_frame(int fd, struct frame_support *fr)
{
	int n;
	unsigned char len;

	if (fr->missing_bytes == 0) {
		/* new frame */
		fr->curr_buf_ptr = fr->buffer;

		do {
			n = read(fd, &len, 1);
		} while (n == -1 && errno == EINTR);

		if (n == 0)
			return 0;

		if (n != 1) /* includes < 0 */
			return -EAGAIN;
		
		debug_printf("read 1st byte --> ");
		if (len > MAX_BUF) {
			fprintf(stderr, "6lo frame length too large, len: %hu\n", len);
			return -1;
		}
		debug_printf("length: %hu\n", len);
	} else {
		debug_printf("Getting missing %d bytes\n", fr->missing_bytes);
		len = fr->missing_bytes;
	}
	n = read(fd, fr->curr_buf_ptr, len);
	if (n < 0)
		return n;

	/* update buffer position */
	fr->curr_buf_ptr = fr->buffer + n;

	/* Check if we got the remaining bytes */
	if (n == len) {
		fr->missing_bytes = 0;
	} else {
		fr->missing_bytes = len - n;
	}
	return 0;
}

/*
 * Attempts to write a 6lochar frame using the given file descriptor
 * (writes: a lenght byte first and the received frame from the NGHAM link)
 * @fd: file descriptor
 * @fr: frame support struct containing the buffer and last position
 *
 * returns:
 * < 0 if an error occurs
 * 0 otherwise (the frame may not have completely written)
 * > 0 corresponding the number of bytes not written
 *
 */
int write_sixlo_frame(int fd, const struct frame_support *fr)
{
	int payload_len, ret;
	const struct ngham_rx *packet;
	unsigned char *sixlo_frame;

	if (ngham_parse(parser, (char*)fr->buffer, frame_length(fr))) {
		fprintf(stderr, "Failed to parse NGHAM packet\n");
		return -1;
	}

	payload_len = ngham_payloadSize(parser);
	if (payload_len <= 0) {
		fprintf(stderr, "Received incorrect NGHAM payload size\n");
		return -1;
	}

	packet = ngham_rxPacket(parser);
	if (!packet) {
		fprintf(stderr, "Failed to extract NGHAM packet (probably not RX)\n");
		return -1;
	}

	if (packet->flags & NGHAMFlag_extension) {
		debug_printf("Skipping NGHAM packet because it has extension data\n");
		return -1;
	}

	sixlo_frame = malloc(payload_len + 1);
	if (!sixlo_frame) {
		fprintf(stderr, "Failed to create sixlo frame\n");
		return -1;
	}
	/* add frame length */
	sixlo_frame[0] = payload_len;
	memcpy(&sixlo_frame[1], packet->payload, payload_len);

	ret = write(fd, sixlo_frame, payload_len + 1);
	if (ret < 0)
		fprintf(stderr, "Failed to write sixlo frame\n");

	debug_printf("Wrote %d bytes to 6lo char\n", ret);

	free(sixlo_frame);
	return (payload_len + 1) - ret;
}


#ifdef OWL_DEV

/*
 * Attempts to read an NGHAM frame using the given file descriptor
 * @fd: file descriptor
 * @fr: support frame with buffer, current buf position and number of missing bytes
 *
 * returns:
 * < 0 if an error occurs
 * 0 otherwise (the frame may still be incomplete)
 *
 */
int read_ngham_frame(int fd, struct frame_support *fr)
{
        int n;
	unsigned char len = 0;

	if (fr->missing_bytes == 0) {
		/* new frame */
		fr->curr_buf_ptr = fr->buffer;
		fr->buffer[0] = 0;

		/* look for NGHAM start byte 0x24 */
	        do {
        		n = read(fd, fr->buffer, 1);
        	} while (n == -1 && errno == EINTR);

                if (n == 0)
                        return 0;
                if (n < 0)
                        return -EAGAIN;
        
                /* maybe the next read... */
                if (fr->buffer[0] != 0x24)
                    return 1;

		/* update buf pos */
		fr->curr_buf_ptr++;

                /* we need the rest of the header */
                fr->missing_bytes -= (OWL_GEN_HEADER-1);

                return 1;
        }

        if (fr->missing_bytes < 0) {
		/* read the rest of the NGHAM header */
		n = read(fd, fr->curr_buf_ptr, -fr->missing_bytes);
		if (n < 0) {
			return -EAGAIN;
                }

                fr->missing_bytes += n;
		/* update buf pos */
		fr->curr_buf_ptr += n;

                /* Still don't have a full header, read more */
                if (fr->missing_bytes < 0)
                    return n;
                /* should never occur!! BUG */
                if (fr->missing_bytes > 0) {
		    fprintf(stderr, "READ MORE THAN THE HEADER --> BUG");
                    return -1;
                }

		debug_printf("read NGHAM header --> ");
		/* payload size in the last byte */
		len = fr->buffer[OWL_GEN_HEADER - 1];
		if (len + OWL_GEN_HEADER > MAX_BUF) {
			fprintf(stderr, "NGHAM frame length too large, len: %hu", len);
			return -1;
		}
		/* header successfully read and frame length seems ok! */
		debug_printf("payload length: %hu\n", len);
	}

        if (fr->missing_bytes > 0) {
		debug_printf("Getting missing %hu bytes\n", fr->missing_bytes);
		len = fr->missing_bytes;
	}

        /* read rest of the packet */
	n = read(fd, fr->curr_buf_ptr, len);
	if (n < 0)
		return n;

	/* update buffer position */
	fr->curr_buf_ptr = fr->curr_buf_ptr + n;

	/* Check if we got the remaining bytes */
	if (n == len) {
                fr->missing_bytes = 0;
	} else {
		fr->missing_bytes = len - n;
	}

	return 0;
}

/*
 * Attempts to write an NGHAM frame using the given file descriptor
 * (writes: the 6lowpan/80215.4 packet into the payload of an NGHAM frame)
 * @fd: file descriptor
 * @fr: frame support struct containing the buffer and last position
 *
 * returns:
 * < 0 if an error occurs
 * 0 if all bytes transmitted (the frame may not have completely written)
 * > 0 corresponding the number of bytes not written
 */
int write_ngham_frame(int fd, const struct frame_support *fr)
{
	int tx_len, ret;
	/* NGHAM SPP headers: start, crc, type, length, flags = 6 bytes */
	size_t req_len = 6 + frame_length(fr);
	char packet[req_len];

	tx_len = ngham_pack(packet, req_len, (char *)fr->buffer, frame_length(fr));
	if (tx_len < 0) {
		fprintf(stderr, "Failed to pack NGHAM packet\n");
		return tx_len;
	}

	ret = write(fd, packet, tx_len);
	if (ret < 0)
		fprintf(stderr, "Failed to write NGHAM packet\n");

	debug_printf("Wrote %d bytes to OWL\n", ret);
	/* returns 0 if all bytes were sent */
	return tx_len - ret;
}


static int configure_owl_serial(int serial)
{
	debug_printf("Configuring serial port\n");
	if (tcflush(serial, TCIOFLUSH))
		return -1;

	struct termios opts;

	debug_printf("Retrieving TTY attributes\n");
	if (tcgetattr(serial, &opts))
		return -1;

	opts.c_iflag = IGNPAR | IGNBRK;
	opts.c_oflag = 0;
	opts.c_cflag &= ~(CSIZE | CSTOPB | PARENB);
	opts.c_cflag |= CS8 | CREAD | CLOCAL;
	opts.c_lflag = 0;

	if (cfsetospeed(&opts, B115200))
		return -1;

	debug_printf("Setting new attributes\n");
	if (tcsetattr(serial, TCSANOW, &opts))
		return -1;

	return 0;
}

#endif



static sigset_t init_signal_handler()
{
	struct sigaction sigAction = { .sa_handler = &set_escape_loop_flag };
	int signals[] = { SIGTERM, SIGINT, SIGHUP };
	sigset_t sigMask;
	sigemptyset(&sigMask);

	for (size_t i = 0; i < sizeof(signals)/sizeof(signals[0]); ++i)
	{
		sigaddset(&sigMask, signals[i]);
		if (sigaction(signals[i], &sigAction, NULL))
		{
			fprintf(stderr, "Failed to initialise signal handler: %s\n",
						strerror(errno));
			exit(EXIT_FAILURE);
		}
	}
	return sigMask;
}

static void set_escape_loop_flag(int signal)
{
	s_signal_caught = signal;
}

#ifdef DEBUG
static void debug_printf(const char *format, ...)
{
	va_list args;
	va_start(args, format);
	vfprintf(stdout, format, args);
	va_end(args);
}
#else
static void debug_printf(const char *format, ...)
{
	(void)format;
}
#endif

