/*
 *   Copyright (C) 2014 by Jonathan Naylor G4KLX and John Hays K7VE
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 *   Adapted by K7VE from G4KLX dv3000d
 *
 *   (2025) Adapted by KD0OSS for Module17 DSTAR hack
 */

#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <unistd.h>
#include <ctype.h>
#include <errno.h>
#include <termios.h>
#include <string.h>
#include <fcntl.h>
#include <stdint.h>
#include <sys/stat.h>

#if defined(RASPBERRY_PI)
#include <wiringPi.h>
int  wiringPiSetup();
void pinMode(int pin, int mode);
void digitalWrite(int pin, int value);
#define OUTPUT  1
#define HIGH    1
#define LOW     0
#endif

#define VERSION "2025-1-7"

void delay(unsigned int delay)
{
	struct timespec tim, tim2;
	tim.tv_sec = 0;
	tim.tv_nsec = delay * 1000UL;
	nanosleep(&tim, &tim2);
};

#define	AMBE3000_HEADER_LEN	    4U
#define	AMBE3000_START_BYTE	    0x61U
#define AMBE3000_TYPE_CONFIG	0x00
#define AMBE3000_PKT_PARITYMODE	0x3f
#define AMBE3000_PKT_RATEP		0x0a

const uint8_t SDV_RESETSOFTCFG_ALAW[11]  = {0x61, 0x00, 0x07, 0x00, 0x34, 0xE8, 0x00, 0x00, 0xE0, 0x00, 0x00};  // Noise suppression Enabled and A-LAW
const uint8_t AMBE3000_PARITY_DISABLE[8] = {AMBE3000_START_BYTE, 0x00, 0x04, AMBE3000_TYPE_CONFIG, AMBE3000_PKT_PARITYMODE, 0x00, 0x2f, 0x14};
const uint8_t AMBE2000_2400_1200[17] = {AMBE3000_START_BYTE, 0x00, 0x0d, AMBE3000_TYPE_CONFIG, AMBE3000_PKT_RATEP, 0x01U, 0x30U, 0x07U, 0x63U, 0x40U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x48U};

#define	BUFFER_LENGTH		    400U

int serialMod17Fd;
int serialDongleFd;
int debugD = 0;
int debugM = 0;
char dongletty[50] = "";
char mod17tty[50] = "";

void dump(char *text, unsigned char *data, unsigned int length)
{
	unsigned int offset = 0U;
	unsigned int i;

	fputs(text, stdout);
	fputc('\n', stdout);

	while (length > 0U) {
		unsigned int bytes = (length > 16U) ? 16U : length;

		fprintf(stdout, "%04X:  ", offset);

		for (i = 0U; i < bytes; i++)
			fprintf(stdout, "%02X ", data[offset + i]);

		for (i = bytes; i < 16U; i++)
			fputs("   ", stdout);

		fputs("   *", stdout);

		for (i = 0U; i < bytes; i++) {
			unsigned char c = data[offset + i];

			if (isprint(c))
				fputc(c, stdout);
			else
				fputc('.', stdout);
		}

		fputs("*\n", stdout);

		offset += 16U;

		if (length >= 16U)
			length -= 16U;
		else
			length = 0U;
	}
	
}

#if defined(RASPBERRY_PI)

int openWiringPi(void)
{
        int ret = wiringPiSetup();
        if (ret == -1) {
                fprintf(stderr, "dv3000d: error when initialising wiringPi\n");
                return 0;
        }

        pinMode(7, OUTPUT);             // Power

        // Reset the hardware
        digitalWrite(7, LOW);

        delay(20UL);

        digitalWrite(7, HIGH);

        delay(750UL);

        if (debug)
                fprintf(stdout, "opened the Wiring Pi library\n");

        return 1;
}

#endif


int openSerial(char *serial)
{
	struct termios tty;
	int fd;
	
	fd = open(serial, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0) {
		fprintf(stderr, "AMBE_Host: error when opening %s, errno=%d\n", serial, errno);
		return fd;
	}

	if (tcgetattr(fd, &tty) != 0) {
		fprintf(stderr, "AMBE_Host: error %d from tcgetattr\n", errno);
		return -1;
	}

	cfsetospeed(&tty, B460800);
	cfsetispeed(&tty, B460800);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
	tty.c_iflag &= ~IGNBRK;
	tty.c_lflag = 0;

	tty.c_oflag = 0;
	tty.c_cc[VMIN]  = 0;
	tty.c_cc[VTIME] = 5;

	tty.c_iflag &= ~(IXON | IXOFF | IXANY);

	tty.c_cflag |= (CLOCAL | CREAD);

	tty.c_cflag &= ~(PARENB | PARODD);
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		fprintf(stderr, "AMBE_Host: error %d from tcsetattr\n", errno);
		return -1;
	}

	if (debugD && debugM)
		fprintf(stdout, "opened %s\n", serial);
	return fd;
}

int processSerialmod17(void)
{
	unsigned char buffer[BUFFER_LENGTH];
	unsigned int respLen;
	unsigned int offset;
	ssize_t len;

	len = read(serialMod17Fd, buffer, 1);
	if (len != 1) {
		fprintf(stderr, "AMBE_Host: error when reading from the serial port, errno=%d\n", errno);
		return 0;
	}

	if (buffer[0U] != AMBE3000_START_BYTE) {
		fprintf(stderr, "AMBE_Host: unknown byte from the Module17, 0x%02X\n", buffer[0U]);
		return 1;
	}

	offset = 0U;
	while (offset < (AMBE3000_HEADER_LEN - 1U)) {
		len = read(serialMod17Fd, buffer + 1U + offset, AMBE3000_HEADER_LEN - 1 - offset);

		if (len == 0)
			delay(5UL);

		offset += len;
	}

	respLen = buffer[1U] * 256U + buffer[2U];

	offset = 0U;
	while (offset < respLen) {
		len = read(serialMod17Fd, buffer + AMBE3000_HEADER_LEN + offset, respLen - offset);

		if (len == 0)
			delay(5UL);

		offset += len;
	}

	respLen += AMBE3000_HEADER_LEN;

	if (debugM)
		dump("Module17 serial data", buffer, respLen);

	len = write(serialDongleFd, buffer, respLen);
	if (len != respLen) {
		fprintf(stderr, "AMBE_Host: error when writing to the dongle serial port, errno=%d\n", errno);
		return 0;
	}

	return 1;
}

int processSerialdongle(void)
{
	unsigned char buffer[BUFFER_LENGTH];
	unsigned int respLen;
	unsigned int offset;
	ssize_t len;

	len = read(serialDongleFd, buffer, 1);
	if (len != 1) {
		fprintf(stderr, "AMBE_Host: error when reading from the serial port, errno=%d\n", errno);
		return 0;
	}

	if (buffer[0U] != AMBE3000_START_BYTE) {
		fprintf(stderr, "AMBE_Host: unknown byte from the DV3000, 0x%02X\n", buffer[0U]);
		return 1;
	}

	offset = 0U;
	while (offset < (AMBE3000_HEADER_LEN - 1U)) {
		len = read(serialDongleFd, buffer + 1U + offset, AMBE3000_HEADER_LEN - 1 - offset);

		if (len == 0)
			delay(5UL);

		offset += len;
	}

	respLen = buffer[1U] * 256U + buffer[2U];

	offset = 0U;
	while (offset < respLen) {
		len = read(serialDongleFd, buffer + AMBE3000_HEADER_LEN + offset, respLen - offset);

		if (len == 0)
			delay(5UL);

		offset += len;
	}

	respLen += AMBE3000_HEADER_LEN;

	if (debugD)
		dump("AMBE dongle serial data", buffer, respLen);

	len = write(serialMod17Fd, buffer, respLen);
	if (len != respLen) {
		fprintf(stderr, "AMBE_Host: error when writing to the Module17 port, errno=%d\n", errno);
		return 0;
	}

	return 1;
}

int main(int argc, char **argv)
{
	int daemon = 0;
	int topFd;
	int commnum;
	fd_set fds;
	int ret;
	int c;

	while ((c = getopt(argc, argv, "d:a:i:vx")) != -1) {
		switch (c) {
			case 'd':
				daemon = 1;
				break;
			case 'a':
				strcpy(dongletty,optarg);
				break;
			case 'i':
				strcpy(mod17tty,optarg);
				break;
			case 'v':
				fprintf(stdout, "AMBE_Host: version " VERSION "\n");
				return 0;
			case 'x':
				debugM = 1;
				debugD = 1;
				break;
			default:
				fprintf(stderr, "Usage: AMBE_Host [-d] [-a Dongle tty] [-i Module17 tty] [-v] [-x]\n");
				return 1;
		}
	}
	if (strlen(dongletty) < 1) {
		fprintf(stderr, "An AMBE dongle tty filename (-i /dev/ttyXXX) must be set.\n");
		return 1;
	}
	if (strlen(mod17tty) < 1) {
		fprintf(stderr, "An Module17 tty filename (-a /dev/ttyXXX) must be set.\n");
		return 1;
	}

	if (daemon) {
		pid_t pid = fork();

		if (pid < 0) {
			fprintf(stderr, "AMBE_Host: error in fork(), exiting\n");
			return 1;
		}

		// If this is the parent, exit
		if (pid > 0)
			return 0;

		// We are the child from here onwards
		setsid();

		umask(0);
	}

#if defined(RASPBERRY_PI)
        ret = openWiringPi();
        if (!ret) {
		fprintf(stderr,"Unable to open WiringPi, exiting\n");
                return 1;
	} else {
		fprintf(stderr,"Reset DV3000\n");
	}

#endif
	serialMod17Fd = openSerial(mod17tty);
	if (serialMod17Fd < 0)
		return 1;

	serialDongleFd = openSerial(dongletty);
		return 1;

	uint8_t buf[6];
	write(serialDongleFd, SDV_RESETSOFTCFG_ALAW, 11);
	sleep(2);
	read(serialDongleFd, buf, 6);
	if (buf[4] != 0x39)
	{
		fprintf(stderr, "Dongle not resetting.\n");
		quit();
	}
	write(serialDongleFd, AMBE3000_PARITY_DISABLE, 8);
	usleep(5000);
	read(serialDongleFd, buf, 6);
	write(serialDongleFd, AMBE2000_2400_1200, 17);
	usleep(5000);
	read(serialDongleFd, buf, 6);

	buf[0] = 0x61;
	buf[1] = 0x03;
	write(serialMod17Fd, buf, 2);

	topFd = serialMod17Fd;
	if (serialDongleFd > topFd)
		topFd = serialDongleFd;
	topFd++;

	for (;;) {
		FD_ZERO(&fds);
		FD_SET(serialMod17Fd, &fds);
		FD_SET(serialDongleFd, &fds);
		ret = select(topFd, &fds, NULL, NULL, NULL);
		if (ret < 0) {
			fprintf(stderr, "AMBE_Host: error from select, errno=%d\n", errno);
			return 1;
		}

		if (FD_ISSET(serialMod17Fd, &fds)) {
			ret = processSerialmod17();
			if (!ret)
				return 1;
		}

		if (FD_ISSET(serialDongleFd, &fds)) {
			ret = processSerialdongle();
			if (!ret)
				return 1;
		}
	}

	return 0;
}
