#ifndef RMDEV_DRIVERS_CAN_H
#define RMDEV_DRIVERS_CAN_H

#include <stdbool.h>
#include <stdint.h>

/* Both on-board TJA1044 transceivers. CAN1 = 2-pin; CAN2 = 4-pin. */
#define CAN_FRC_BIT_RATE_HZ 1000000UL

typedef enum { CAN_BUS_1 = 0, CAN_BUS_2 = 1 } can_bus_t;
typedef struct {
    can_bus_t bus;
    uint32_t identifier;
    uint8_t length;
    uint8_t data[8];
} can_frame_t;

bool can_init(uint32_t apb1_clock_hz, uint32_t bit_rate);
bool can_receive(can_bus_t bus, can_frame_t *frame);
bool can_transmit(const can_frame_t *frame);

#endif /* RMDEV_DRIVERS_CAN_H */
