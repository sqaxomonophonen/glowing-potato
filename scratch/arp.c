#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <unistd.h>
#include <time.h>
#include <math.h>

static inline float freq2inc(float freq, float clk)
{
	return freq * (float)(1<<20) / clk;
}

static inline float note2freq(float note)
{
	return 440.0f * powf(2.0f, note*(1.0f/12.0f));
}

static void encode_buf(char* buf, float freq, int clksrc)
{
	float clk = clksrc ? 500e3f : 32e3f;
	int inc = freq2inc(freq, clk);
	const int max = (1<<13)-1;
	if (inc < 0 || inc > max) {
		printf("WARNING: inc=%d is out of range\n", inc);
		if (inc < 0) inc = 0;
		if (inc > max) inc = max;
	}
	buf[0] = inc & 0xff;
	buf[1] = ((inc >> 8) & 0x1f) | (clksrc ? 0x20 : 0);
	buf[2] = 0;
}

static void nsleep(long duration)
{
	struct timespec req;
	req.tv_sec = 0;
	req.tv_nsec = duration;
	nanosleep(&req, NULL);
}

#define ARRAY_LENGTH(xs) (sizeof(xs) / sizeof(xs[0]))

int main(int argc, char** argv)
{
	int fd = open("/dev/i2c-4", O_RDWR);
	if (fd == -1) {
		perror(argv[1]);
		exit(EXIT_FAILURE);
	}

	const int address = 0x10;

	char buf[3];

	const float arp_notes[] = {0.0f, 3.0f, 7.0f};
	int arp_index = 0;

	for (;;) {
		encode_buf(buf, note2freq(arp_notes[arp_index] + 20.0f), 1);

		arp_index = (arp_index + 1) % ARRAY_LENGTH(arp_notes);

		struct i2c_msg msg = {
			.addr = address,
			.flags = 0,
			.len = 3,
			.buf = buf,
		};
		struct i2c_rdwr_ioctl_data data = {
			.msgs = &msg,
			.nmsgs = 1,
		};

		if (ioctl(fd, I2C_RDWR, &data) == -1) {
			perror(argv[1]);
			exit(EXIT_FAILURE);
		}

		nsleep(1e9 / 50.0f);
	}

	close(fd);

	return EXIT_SUCCESS;
}

