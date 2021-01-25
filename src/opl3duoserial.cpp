/*
 * AdPlug - Replayer for many OPL2/OPL3 audio file formats.
 * Copyright (C) 1999 - 2005 Simon Peter <dn.tlp@gmx.net>, et al.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * opl3duoserial.cpp - Serial interface to an OPL3 Duo!, by Kevin Vance <kvance@kvance.com>
 */

#include <cstdint>
#include <cstdio>
#include <ctime>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "opl3duoserial.h"

// There is no significance to this number, determined experimentally by what didn't make
// annoying clicking sounds
static const long MIN_WRITE_DELAY = 225000;  // nanoseconds


CSerialDuoOpl::CSerialDuoOpl(const char *device, int baud) :
	device(device), baud(baud), fd(-1)
{
	currType = TYPE_DUAL_OPL3;

	init();
}

CSerialDuoOpl::~CSerialDuoOpl()
{
	reset(ReturnBeforeComplete);
	if(fd >= 0) {
		close(fd);
	}
}

void CSerialDuoOpl::init()
{
	if(fd >= 0) {
		printf("ALREADY INITIALIZED\n");
		return;
	}
	currChip = 0;

	// Open serial port
	fd = open(device, O_RDWR);
	if(fd < 0) {
		return;
	}

	// Read existing settings
	bool success = true;
	struct termios tty;
	if(tcgetattr(fd, &tty) != 0) {
		success = false;
	}

	// Update settings
	if(success) {
		tty.c_cflag &= ~CSIZE;		// clear all size bits
		tty.c_cflag |= CS8;		// 8 bit
		tty.c_cflag &= ~PARENB;		// no parity
		tty.c_cflag &= ~CSTOPB;		// 1 stop bit
		tty.c_cflag &= ~CRTSCTS;	// disable hw flow control
		tty.c_cflag |= CREAD;		// enable read
		tty.c_cflag |= CLOCAL;		// ignore control lines

		tty.c_lflag &= ~ICANON;		// disable canonical mode
		tty.c_lflag &= ~ISIG;		// disable signal characters

		tty.c_iflag &= ~(IXON | IXOFF | IXANY);	// disable sw flow control

		// disable special character handling
		tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

		tty.c_oflag &= ~OPOST;		// disable special handling
		tty.c_oflag &= ~ONLCR;		// disable newline conversion

		// return from read immediately
		tty.c_cc[VTIME] = 0;
		tty.c_cc[VMIN] = 0;

		// set baud rate
		cfsetspeed(&tty, baud);

		if(tcsetattr(fd, TCSANOW, &tty) != 0) {
			success = false;
		}
	}

	if(success) {
		// Wait for acknowledgement
		char b;
		while(read(fd, &b, 1) < 1 && b != 'k');
		printf("READY\n");
		// Send reset hardware message
		reset();
	} else {
		// Clean up after failure
		printf("FAILED\n");
		close(fd);
		fd = -1;
	}
}

void CSerialDuoOpl::reset(enum ResetMode mode)
{
	if(fd < 0) {
		return;
	}
	const uint8_t buffer = 0xff;  // high bit is the only one that matters
	int n = ::write(fd, &buffer, 1);
	printf("[%d] reset\n", n);

	if(mode == WaitUntilComplete) {
		struct timespec resetslp = {1, 500000000};
		nanosleep(&resetslp, NULL);
	}

	// Reset the last write time
	lastWrite.tv_sec = 0;
	lastWrite.tv_nsec = 0;
}

void CSerialDuoOpl::write(int reg, int val)
{
	// Wait if it's too soon after the last write
	struct timespec now;
	clock_gettime(CLOCK_MONOTONIC, &now);
	struct timespec diff = diffTime(now, lastWrite);
	if(diff.tv_sec == 0) {
		diff.tv_nsec = MIN_WRITE_DELAY - diff.tv_nsec;
		if(diff.tv_nsec > 0) {
			nanosleep(&diff, NULL);
		}
	}

	// | Byte 1   | Byte 2?  | Byte 3?  |
	// | RC.....r | rrrrrrrr | vvvvvvvv |
	//
	// If the high bit of byte 1 is set, reset the hardware and stop reading.
	// Otherwise, interpret the rest of byte 1:
	// C = chip select
	// r = 9th bit of register
	// Then read another byte with the lower 8 bits of the register.
	// And read the last byte with the value to set.

	const uint8_t buffer[] = {
		uint8_t(((currChip & 1) << 6) |	// chip select
		        ((reg & 0x100)) >> 8),	// register high bit
		uint8_t(reg & 0xff),		// register low bits
		uint8_t(val),			// value
	};
	int n = ::write(fd, buffer, sizeof(buffer));

	lastWrite = now;
}

struct timespec CSerialDuoOpl::diffTime(const struct timespec &t1, struct timespec t2)
{
	/* Perform the carry for the later subtraction by updating y. */
	if (t1.tv_nsec < t2.tv_nsec) {
		long nsec = (t2.tv_nsec - t1.tv_nsec) / 1000000000 + 1;
		t2.tv_nsec -= 1000000000 * nsec;
		t2.tv_sec += nsec;
	}
	if (t1.tv_nsec - t2.tv_nsec > 1000000) {
		long nsec = (t1.tv_nsec - t2.tv_nsec) / 1000000000;
		t2.tv_nsec += 1000000000 * nsec;
		t2.tv_sec -= nsec;
	}

	/* Compute the time remaining to wait.
	   tv_nsec is certainly positive. */
	struct timespec result;
	result.tv_sec = t1.tv_sec - t2.tv_sec;
	result.tv_nsec = t1.tv_nsec - t2.tv_nsec;
	return result;
}
