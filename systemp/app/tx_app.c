/* tx_app.c */
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include "speed_proto.h"

int main(void) {
    int fd = open("/dev/speed_tx", O_RDWR);
    if (fd < 0) {
        perror("open speed_tx");
        return 1;
    }

    printf("Press ENTER to send ACCEL +10 command\n");
    while (1) {
        if (getchar() == '\n') {
            int8_t delta = 10;
            if (write(fd, &delta, 1) != 1) {
                perror("write");
                break;
            }
            printf("Sent ACCEL +10\n");

            speed_frame_t ack;
            ssize_t n = read(fd, &ack, FRAME_LEN);
            if (n == FRAME_LEN && calc_checksum(&ack) == ack.checksum) {
                printf("ACK: start=0x%02X, status=0x%02X, speed=%d\n",
                       ack.start, ack.type, ack.value);
            } else {
                fprintf(stderr, "ACK error\n");
            }
        }
    }

    close(fd);
    return 0;
}
