/* rx_app.c */
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include "speed_proto.h"

int main(void) {
    int fd = open("/dev/speed_rx", O_RDONLY);
    if (fd < 0) {
        perror("open speed_rx");
        return 1;
    }

    speed_frame_t frame;
    while (1) {
        ssize_t n = read(fd, &frame, FRAME_LEN);
        if (n == FRAME_LEN && calc_checksum(&frame) == frame.checksum) {
            printf("RX: start=0x%02X, type=0x%02X, value=%d\n",
                   frame.start, frame.type, frame.value);
        } else {
            fprintf(stderr, "Invalid frame or checksum error\n");
        }
    }

    close(fd);
    return 0;
}
