#!/usr/bin/python

import os
import mmap
import cairo


class ssd1322:
    def __init__(self):
        # open control fifo
        self.fifo_fd = os.open('/run/ssd1322/fifo', os.O_WRONLY | os.O_NONBLOCK)

        # open ssd1322's framebuffer memory
        with open('/dev/shm/ssd1322', 'r+b') as mem_fd:
            self.mm = mmap.mmap(mem_fd.fileno(), 65536, mmap.MAP_SHARED)

        # create cairo surface from ssd1322's shared memory
        self.surface = cairo.ImageSurface.create_for_data(self.mm, cairo.FORMAT_RGB24, 256, 64)

    def close(self):
        os.close(self.fifo_fd)
        self.surface.finish()
        self.mm.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def reset(self):
        os.write(self.fifo_fd, '\0')

    def disable(self):
        os.write(self.fifo_fd, '\1')

    def enable(self):
        os.write(self.fifo_fd, '\2')

    def update(self):
        os.write(self.fifo_fd, '\3')

    def dump(self, path):
        self.surface.write_to_png(path)


if __name__ == '__main__':
    with ssd1322() as display:
        display.dump('/dev/shm/ssd1322.png')
