#ifndef SPEED_PROTO_H
#define SPEED_PROTO_H

#include <stdint.h>

/* Start bytes */
#define TX_START_BYTE 0xA5
#define RX_START_BYTE 0x5A

/* Command codes */
enum speed_cmd_e {
    CMD_ACCEL = 0x01,
    CMD_DECEL = 0x02,
    CMD_STOP  = 0x03,
};

/* Status codes */
enum speed_status_e {
    STATUS_NORMAL  = 0x00,
    STATUS_ACCEL   = 0x01,
    STATUS_LIMITED = 0x02,
    STATUS_IDLE    = 0x03,
};

/* Frame length */
#define FRAME_LEN 4

/* Protocol frame */
#pragma pack(push, 1)
typedef struct {
    uint8_t start;     /* Start byte */
    uint8_t type;      /* CMD or STATUS */
    int8_t  value;     /* Signed value: speed delta or current speed */
    uint8_t checksum;  /* (start + type + value) & 0xFF */
} speed_frame_t;
#pragma pack(pop)

/* Calculate checksum */
static inline uint8_t calc_checksum(const speed_frame_t *f) {
    uint16_t sum = f->start + f->type + (uint8_t)f->value;
    return (uint8_t)(sum & 0xFF);
}

#endif /* SPEED_PROTO_H */
