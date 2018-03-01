/*
 *  Copyright (C) 2017,2018 Raman Shyshniou <rommer@ibuffed.com>
 *  All Rights Reserved.
 *
 *  This is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This software is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this software; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307,
 *  USA.
 */

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <limits.h>
#include <errno.h>
#include <time.h>

#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#define true 1
#define false 0

#define SPI_DEV   "/dev/spidev0.0"  /* spi device */
#define GPIO_RES  115               /* reset pin */
#define GPIO_DC   116               /* d/c pin */
#define GPIO_EN   88                /* led power enable pin */

static int spi_fd;
static int res_fd;
static int dc_fd;
static int en_fd;
static int dc;

/* FIXME: mode 3 a lot more stable for ssd1322 */
uint8_t spi_mode = SPI_MODE_3 | SPI_NO_CS;
uint8_t spi_bits = 8;
int spi_rate = 16000000;
int init[] = {                  /* Initialization for SSD1322 OLED display, 256x64 */
    -1, 0xFD, 0x12,             /* Unlock OLED driver IC */
    -1, 0xAE,                   /* Display OFF (blank) */
    -1, 0xB3, 0xF3,             /* Display divide clockratio/frequency */
    -1, 0xCA, 0x3F,             /* Multiplex ratio, 1/64, 64 COMS enabled */
    -1, 0xA2, 0x00,             /* Set offset, the display map starting line is COM0 */
    -1, 0xAB, 0x01,             /* Select internal VDD */
    -1, 0xA1, 0x00,             /* Set start line position */
    -1, 0xA0, 0x14, 0x11,       /* Set remap, horiz address increment, disable colum address remap, */
                                /* enable nibble remap, scan from com[N-1] to COM0, disable COM split odd even */
    -1, 0xB4, 0xA0, 0xFD,       /* Display enhancement A, external VSL, enhanced low GS display quality */
    -1, 0xC1, 0x7F,             /* Contrast current, 256 steps, default is 0x7F */
    -1, 0xC7, 0x0F,             /* Master contrast current, 16 steps, default is 0x0F */
    -1, 0xB1, 0xF0,             /* Phase Length */
    -1, 0xD1, 0x82, 0x20        /* Display enhancement B */
    -1, 0xBB, 0x0D,             /* Pre-charge voltage */
    -1, 0xBE, 0x00,             /* Set VCOMH */
    -1, 0xA6,                   /* Normal display */
//  -1, 0xAF,                   /* Display ON */
    -2 };

static int fifo_fd;
static uint32_t *shm;
static uint8_t *buf;
static uint8_t *pan0;
static uint8_t *pan1;


static void error(char *msg, int err)
{
    if (err)
        fprintf(stderr, "%s: %s\n", msg, strerror(err));
    else
        fprintf(stderr, "%s\n", msg);

    exit(1);
}


static inline uint8_t rgb2gray(uint32_t rgb)
{
    return (rgb >> 18 & 0x3F) + (rgb >> 9 & 0x7F) + (rgb >> 2 & 0x3F);
}


static int gpio_open(unsigned n)
{
    char path[PATH_MAX];
    int fd;

    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%u/direction", n);
    if ((fd = open(path, O_WRONLY)) < 0) {
        char data[16];

        if (errno != ENOENT)
            return -1;

        if ((fd = open("/sys/class/gpio/export", O_WRONLY)) < 0)
            return -1;

        snprintf(data, sizeof(data), "%u", n);

        if (write(fd, data, strlen(data)) < 0)
            return -1;

        if (close(fd) < 0)
            return -1;

        if ((fd = open(path, O_WRONLY)) < 0)
            return -1;
    }

    if (write(fd, "out", 3) < 0)
        return -1;

    if (close(fd) < 0)
        return -1;

    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%u/value", n);
    if ((fd = open(path, O_WRONLY)) < 0)
        return -1;

    if (write(fd, "0", 1) < 0)
        return -1;

    return fd;
}


static int spi_write(int fd, void *data, size_t count)
{
    struct spi_ioc_transfer spi;

    spi.tx_buf        = (unsigned long) data;
    spi.rx_buf        = 0;
    spi.len           = count;
    spi.delay_usecs   = 0;
    spi.speed_hz      = spi_rate;
    spi.bits_per_word = spi_bits;

    return ioctl(fd, SPI_IOC_MESSAGE(1), &spi);
}


static int spi_write_command(int fd, uint8_t command)
{
    if (dc && write(dc_fd, "0", 1) < 0)
        return -1;

    dc = 0;

    return spi_write(fd, &command, 1);
}


static int spi_write_data(int fd, uint8_t data)
{
    if (!dc && write(dc_fd, "1", 1) < 0)
        return -1;

    dc = 1;

    return spi_write(fd, &data, 1);
}


static int spi_write_data2(int fd, uint8_t data1, uint8_t data2)
{
    uint8_t data[2] = { data1, data2 };

    if (!dc && write(dc_fd, "1", 1) < 0)
        return -1;

    dc = 1;

    return spi_write(fd, &data, 2);
}


static int ssd1322_write_ram(int fd, void *data, size_t count)
{
    int res;

    if ((res = spi_write_command(fd, 0x5C)) < 0) // SSD1322: Write to RAM enable.
        return res;

    if (!dc && write(dc_fd, "1", 1) < 0)
        return -1;

    dc = 1;

    return spi_write(fd, data, count);
}


static int ssd1322_init(int fd)
{
    int i;

    for (i = 0; i >= 0;)
        switch (init[i]) {
            case -1:
                if (spi_write_command(spi_fd, init[++i]) < 0)
                    return -1;

                while (init[++i] >= 0)
                    if (spi_write_data(spi_fd, init[i]) < 0)
                        return -1;

                continue;

            case -2:
                i = -1;
                break;

            default:
                error("Bad init sequence", 0);
        }

    return 0;
}


static int ssd1322_setcols(int fd, int width)
{
    int offset = (480 - width) / 8;

    if (spi_write_command(spi_fd, 0x15) < 0) // SSD1322: Set start & end cols.
        return -1;

    if (spi_write_data2(spi_fd, offset, offset + (width / 4) - 1) < 0)
        return -1;

    return 0;
}


static int ssd1322_setrows(int fd, int start, int end)
{
    if (spi_write_command(spi_fd, 0x75) < 0) // SSD1322: Set start & end rows.
        return -1;

    if (spi_write_data2(spi_fd, start, end) < 0)
        return -1;

    return 0;
}


static int ssd1322_setstartline(int fd, int start)
{
    if (spi_write_command(spi_fd, 0xA1) < 0) // SSD1322: Set RAM start line.
        return -1;

    if (spi_write_data(spi_fd, start) < 0)
        return -1;

    return 0;
}


static int ssd1322_enable(int fd, int enable)
{
    if (enable) {
        if (spi_write_command(spi_fd, 0xAF) < 0)
            return -1;
    } else {
        if (spi_write_command(spi_fd, 0xAE) < 0)
            return -1;
    }

    return 0;
}


int main(void)
{
    int mmfd, enabled, pan;
    uint8_t cmd;
    mode_t um;
    int res;

    if ((spi_fd = open(SPI_DEV, O_RDWR)) < 0)
        error("Unable to open SPI device", errno);

    if (ioctl(spi_fd, SPI_IOC_WR_MODE, &spi_mode) < 0)
        error("SPI Mode Change failure", errno);

    if (ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bits) < 0)
        error("SPI BPW Change failure", errno);

    if (ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_rate) < 0)
        error("SPI Speed Change failure", errno);

    if ((res_fd = gpio_open(GPIO_RES)) < 0)
        error("Unable to open res gpio", errno);

    if ((dc_fd = gpio_open(GPIO_DC)) < 0)
        error("Unable to open dc gpio", errno);

    if ((en_fd = gpio_open(GPIO_EN)) < 0)
        error("Unable to open en gpio", errno);

    if (mkdir("/run/ssd1322", 0777) < 0 && errno != EEXIST)
        error("Unable to create runtime dir", errno);

    um = umask(0);

    if (mkfifo("/run/ssd1322/fifo", 0666) && errno != EEXIST)
        error("Unable to create fifo", errno);

    if ((fifo_fd = open("/run/ssd1322/fifo", O_RDWR, 0666)) < 0)
        error("Unable to open fifo", errno);

    if ((mmfd = open("/dev/shm/ssd1322", O_CREAT|O_RDWR, 0666)) < 0)
        error("Unable to open shm file", errno);

    umask(um);

    if (ftruncate(mmfd, 65536) < 0)
        error("Unable to truncate shm file", errno);

    if ((shm = mmap(NULL, 65536, PROT_READ|PROT_WRITE, MAP_SHARED, mmfd, 0)) == (void *) -1)
        error("Unable to mmap shm file", errno);

    if (close(mmfd) < 0)
        error("Unable to close shm file", errno);

    if ((buf = mmap(NULL, 8192, PROT_READ|PROT_WRITE, MAP_PRIVATE|MAP_ANONYMOUS, -1, 0)) == (void *) -1)
        error("Unable to allocate buffer", errno);

    if ((pan0 = mmap(NULL, 8192, PROT_READ|PROT_WRITE, MAP_PRIVATE|MAP_ANONYMOUS, -1, 0)) == (void *) -1)
        error("Unable to allocate pan0 buffer", errno);

    if ((pan1 = mmap(NULL, 8192, PROT_READ|PROT_WRITE, MAP_PRIVATE|MAP_ANONYMOUS, -1, 0)) == (void *) -1)
        error("Unable to allocate pan1 buffer", errno);

    enabled = true;
    pan = 0;
    dc = 0;

    /* Reset display at start */
    cmd = 0;

    /* Start read the fifo commands and update screen using shm memory */
    do {
        switch (cmd) {
            /* Reset ssd1322 */
            case 0:
                write(res_fd, "0", 1);
                usleep(300);
                write(res_fd, "1", 1);
                usleep(300);

                /* Init ssd1322 */
                if (ssd1322_init(spi_fd) < 0)
                    error("Unable to init display", errno);

                /* Set width to 256 pixels */
                if (ssd1322_setcols(spi_fd, 256) < 0)
                    error("Unable to init display width", errno);

                /* Clear screen */
                memset(buf, 0, 8192);
                memset(pan0, 0, 8192);
                memset(pan1, 0, 8192);

                if (ssd1322_setrows(spi_fd, 0, 63) < 0)
                    error("Unable to set rows 0..63", errno);

                if (ssd1322_write_ram(spi_fd, buf, 8192) < 0)
                    error("SPI Write failed", errno);

                /* Enable */
                if (enabled) {
                    if (ssd1322_enable(spi_fd, 1) < 0)
                        error("Unable to enable ssd1322", errno);

                    if (write(en_fd, "1", 1) < 0)
                        error("Unable to enable display", errno);
                }

                pan = 0;

                break;

            /* Disable ssd1322 */
            case 1:
                if (enabled) {
                    /* Disable display */
                    if (write(en_fd, "0", 1) < 0)
                        error("Unable to disable display", errno);

                    /* Disable ssd1322 */
                    if (ssd1322_enable(spi_fd, 0) < 0)
                        error("Unable to disable ssd1322", errno);

                    enabled = false;
                }

                break;

            /* Enable ssd1322 */
            case 2:
                if (!enabled) {
                    /* Enable ssd1322 */
                    if (ssd1322_enable(spi_fd, 1) < 0)
                        error("Unable to enable ssd1322", errno);

                    /* Enable display */
                    if (write(en_fd, "1", 1) < 0)
                        error("Unable to enable display", errno);

                    enabled = true;
                }

                break;

            /* Update screen */
            case 3: {
                    uint8_t *p;
                    int i, j, ys, ye;
#ifdef DEBUG
                    unsigned long long nsecs;
                    struct timespec ts, te;

                    clock_gettime(CLOCK_MONOTONIC, &ts);
#endif

                    /* Convert 24-bit RGB to display format */
                    for (i = 0, j = 0; i < 8192; i++) {
                        buf[i] = rgb2gray(shm[j++]) & 0xf0;
                        buf[i] |= rgb2gray(shm[j++]) >> 4;
                    }

                    /* Check start and end lines */
                    p = pan ? pan0 : pan1;

                    for (ys = 0; ys < 64; ys++)
                        if (memcmp(buf + (ys << 7), p + (ys << 7), 128))
                            break;

                    if (ys < 64) {
                        for (ye = 63; ye > ys; ye--)
                            if (memcmp(buf + (ye << 7), p + (ye << 7), 128))
                                break;

                        memcpy(p + (ys << 7), buf + (ys << 7), (1 + ye - ys) << 7);

                        /* Set rows to write */
                        if (pan) {
                            if (ssd1322_setrows(spi_fd, ys, ye) < 0)
                                error("Unable to set pan0 rows", errno);
                        } else {
                            if (ssd1322_setrows(spi_fd, ys + 64, ye + 64) < 0)
                                error("Unable to set pan1 rows", errno);
                        }

                        /* Write data to RAM */
                        if (ssd1322_write_ram(spi_fd, buf + (ys << 7), (1 + ye - ys) << 7) < 0)
                            error("SPI Write failed", errno);
                    }

                    /* Switch display pan */
                    if (pan) {
                        pan = 0;
                        if (ssd1322_setstartline(spi_fd, 0) < 0)
                            error("Unable to set RAM start line.", errno);
                    } else {
                        pan = 1;
                        if (ssd1322_setstartline(spi_fd, 64) < 0)
                            error("Unable to set RAM start line.", errno);
                    }
#ifdef DEBUG
                    clock_gettime(CLOCK_MONOTONIC, &te);
                    nsecs = (te.tv_sec - ts.tv_sec) * 1000000000 +
                            (te.tv_nsec - ts.tv_nsec);

                    fprintf(stderr, "Update time: %.2f ms, wrote %d lines\n",
                            (double)nsecs / 1000000.0, ys < 64 ? ye - ys + 1 : 0);
#endif
                }
                break;

#ifdef DEBUG
            case 9: {
                    /* Custom command */
                    uint8_t cnt;  // data len
                    uint8_t cmd;  // command
                    int i;

                    read(fifo_fd, &cnt, 1);
                    read(fifo_fd, &cmd, 1);

                    if (spi_write_command(spi_fd, cmd) < 0)
                        fprintf(stderr, "Wrote custom command %02x: %s\n", (int) cmd, strerror(errno));
                    else
                        fprintf(stderr, "Wrote custom command %02x: OK\n", (int) cmd);

                    for (i = 0; i < cnt; i++) {
                        uint8_t data;
                        read(fifo_fd, &data, 1);

                        if (spi_write_data(spi_fd, data) < 0)
                            fprintf(stderr, "Wrote custom data %02x: %s\n", (int) data, strerror(errno));
                        else
                            fprintf(stderr, "Wrote custom data %02x: OK\n", (int) data);
                    }
                };
                break;
#endif

            default:
                fprintf(stderr, "Unknown command %02x\n", cmd);
        }

        for (;;) {
            res = read(fifo_fd, &cmd, 1);

            if (res < 0) {
                if (errno != EINTR)
                    error("FIFO read failure", errno);
            } else
                break;
        }

    } while (1);

    return 0;
}
