#include "can.h"
#include "board_io.h"

/* STM32F407/427 bxCAN and GPIO register addresses. */
#define RCC_AHB1ENR       (*(volatile uint32_t *)0x40023830UL)
#define RCC_APB1ENR       (*(volatile uint32_t *)0x40023840UL)
#define CAN1_BASE         0x40006400UL
#define CAN2_BASE         0x40006800UL

#define GPIO_MODER(port)  (*(volatile uint32_t *)((port) + 0x00UL))
#define GPIO_OTYPER(port) (*(volatile uint32_t *)((port) + 0x04UL))
#define GPIO_OSPEEDR(port) (*(volatile uint32_t *)((port) + 0x08UL))
#define GPIO_PUPDR(port)  (*(volatile uint32_t *)((port) + 0x0CUL))
#define GPIO_AFRL(port)   (*(volatile uint32_t *)((port) + 0x20UL))
#define GPIO_AFRH(port)   (*(volatile uint32_t *)((port) + 0x24UL))

#define CAN_MCR(base)     (*(volatile uint32_t *)((base) + 0x00UL))
#define CAN_MSR(base)     (*(volatile uint32_t *)((base) + 0x04UL))
#define CAN_BTR(base)     (*(volatile uint32_t *)((base) + 0x1CUL))
#define CAN_TSR(base)     (*(volatile uint32_t *)((base) + 0x08UL))
#define CAN_RF0R(base)    (*(volatile uint32_t *)((base) + 0x0CUL))
#define CAN_RIR(base)     (*(volatile uint32_t *)((base) + 0x1B0UL))
#define CAN_RDTR(base)    (*(volatile uint32_t *)((base) + 0x1B4UL))
#define CAN_RDLR(base)    (*(volatile uint32_t *)((base) + 0x1B8UL))
#define CAN_RDHR(base)    (*(volatile uint32_t *)((base) + 0x1BCUL))
#define CAN_FMR           (*(volatile uint32_t *)(CAN1_BASE + 0x200UL))
#define CAN_FM1R          (*(volatile uint32_t *)(CAN1_BASE + 0x204UL))
#define CAN_FS1R          (*(volatile uint32_t *)(CAN1_BASE + 0x20CUL))
#define CAN_FFA1R         (*(volatile uint32_t *)(CAN1_BASE + 0x214UL))
#define CAN_FA1R          (*(volatile uint32_t *)(CAN1_BASE + 0x21CUL))
#define CAN_FILTER_FR1(bank) (*(volatile uint32_t *)(CAN1_BASE + 0x240UL + ((bank) * 8UL)))
#define CAN_FILTER_FR2(bank) (*(volatile uint32_t *)(CAN1_BASE + 0x244UL + ((bank) * 8UL)))

#define RCC_AHB1ENR_GPIOBEN (1UL << 1)
#define RCC_AHB1ENR_GPIODEN (1UL << 3)
#define RCC_APB1ENR_CAN1EN  (1UL << 25)
#define RCC_APB1ENR_CAN2EN  (1UL << 26)

#define CAN_MCR_INRQ        (1UL << 0)
#define CAN_MCR_SLEEP       (1UL << 1)
#define CAN_MCR_ABOM        (1UL << 6)
#define CAN_MSR_INAK        (1UL << 0)
#define CAN_FMR_FINIT       (1UL << 0)
#define CAN_FMR_CAN2SB_Pos  8U

#define CAN_INIT_TIMEOUT 1000000UL

static void configure_can_pin_pair(uint32_t port, uint8_t rx_pin, uint8_t tx_pin)
{
    const uint32_t pins = (1UL << rx_pin) | (1UL << tx_pin);
    const uint32_t mode_mask = (3UL << (rx_pin * 2U)) | (3UL << (tx_pin * 2U));
    const uint32_t af_mask = (0xFUL << ((rx_pin % 8U) * 4U)) |
                             (0xFUL << ((tx_pin % 8U) * 4U));
    const uint32_t af_value = (9UL << ((rx_pin % 8U) * 4U)) |
                              (9UL << ((tx_pin % 8U) * 4U));
    volatile uint32_t *const af_register =
        (rx_pin < 8U) ? &GPIO_AFRL(port) : &GPIO_AFRH(port);

    /* Alternate function 9 is CAN1/CAN2 on these four board pins. */
    GPIO_MODER(port) = (GPIO_MODER(port) & ~mode_mask) |
                       (2UL << (rx_pin * 2U)) | (2UL << (tx_pin * 2U));
    GPIO_OTYPER(port) &= ~pins; /* push-pull */
    GPIO_OSPEEDR(port) |= mode_mask; /* high speed for clean CAN edges */
    GPIO_PUPDR(port) = (GPIO_PUPDR(port) & ~mode_mask) | (1UL << (rx_pin * 2U));
    *af_register = (*af_register & ~af_mask) | af_value;
}

static bool wait_for_bit(uint32_t base, uint32_t mask, bool set)
{
    for (uint32_t timeout = CAN_INIT_TIMEOUT; timeout != 0U; --timeout) {
        if (((CAN_MSR(base) & mask) != 0U) == set) {
            return true;
        }
    }
    return false;
}

static bool calculate_btr(uint32_t apb1_clock_hz, uint32_t bit_rate, uint32_t *btr)
{
    if ((apb1_clock_hz == 0U) || (bit_rate == 0U) || (btr == 0)) {
        return false;
    }

    /* Prefer 16 time quanta per bit; use the closest exact lower value. */
    for (uint32_t tq = 16U; tq >= 8U; --tq) {
        const uint32_t denominator = bit_rate * tq;
        if ((apb1_clock_hz % denominator) != 0U) {
            continue;
        }

        const uint32_t prescaler = apb1_clock_hz / denominator;
        const uint32_t bs1 = (tq * 4U) / 5U - 1U;
        const uint32_t bs2 = tq - 1U - bs1;
        if ((prescaler >= 1U) && (prescaler <= 1024U) &&
            (bs1 >= 1U) && (bs1 <= 16U) && (bs2 >= 1U) && (bs2 <= 8U)) {
            *btr = (prescaler - 1U) | ((bs1 - 1U) << 16U) | ((bs2 - 1U) << 20U);
            return true;
        }
    }
    return false;
}

static bool initialise_controller(uint32_t base, uint32_t btr)
{
    CAN_MCR(base) = CAN_MCR_INRQ | CAN_MCR_ABOM;
    if (!wait_for_bit(base, CAN_MSR_INAK, true)) {
        return false;
    }

    CAN_BTR(base) = btr;
    CAN_MCR(base) &= ~CAN_MCR_INRQ;
    return wait_for_bit(base, CAN_MSR_INAK, false);
}

static void configure_filters(void)
{
    const uint32_t can1_filter = 0U;
    const uint32_t can2_filter = 14U;
    const uint32_t filters = (1UL << can1_filter) | (1UL << can2_filter);

    CAN_FMR |= CAN_FMR_FINIT;
    CAN_FMR = (CAN_FMR & ~(0x3FUL << CAN_FMR_CAN2SB_Pos)) |
              (14UL << CAN_FMR_CAN2SB_Pos) | CAN_FMR_FINIT;
    CAN_FA1R &= ~filters;
    CAN_FM1R &= ~filters;  /* mask mode */
    CAN_FS1R |= filters;   /* 32-bit scale */
    CAN_FFA1R &= ~filters; /* FIFO 0 */
    CAN_FILTER_FR1(can1_filter) = 0U;
    CAN_FILTER_FR2(can1_filter) = 0U;
    CAN_FILTER_FR1(can2_filter) = 0U;
    CAN_FILTER_FR2(can2_filter) = 0U;
    CAN_FA1R |= filters;   /* zero mask accepts all identifiers */
    CAN_FMR &= ~CAN_FMR_FINIT;
}

bool can_init(uint32_t apb1_clock_hz, uint32_t bit_rate)
{
    uint32_t btr;
    if (!calculate_btr(apb1_clock_hz, bit_rate, &btr)) {
        return false;
    }

    RCC_AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIODEN;
    (void)RCC_AHB1ENR;
    configure_can_pin_pair(BOARD_CAN1_RX_PORT, BOARD_CAN1_RX_PIN, BOARD_CAN1_TX_PIN);
    configure_can_pin_pair(BOARD_CAN2_RX_PORT, BOARD_CAN2_RX_PIN, BOARD_CAN2_TX_PIN);

    /* CAN2 shares CAN1's filter/clock domain, so CAN1 must be enabled first. */
    RCC_APB1ENR |= RCC_APB1ENR_CAN1EN | RCC_APB1ENR_CAN2EN;
    (void)RCC_APB1ENR;
    configure_filters();

    return initialise_controller(CAN1_BASE, btr) && initialise_controller(CAN2_BASE, btr);
}

bool can_receive(can_bus_t bus, can_frame_t *frame)
{
    if ((frame == 0) || (bus > CAN_BUS_2)) return false;
    const uint32_t base = bus == CAN_BUS_1 ? CAN1_BASE : CAN2_BASE;
    if ((CAN_RF0R(base) & 3UL) == 0U) return false;
    const uint32_t rir = CAN_RIR(base);
    const uint32_t dlr = CAN_RDLR(base);
    const uint32_t dhr = CAN_RDHR(base);
    frame->bus = bus;
    frame->extended = (rir & (1UL << 2)) != 0U;
    frame->identifier = frame->extended ? (rir >> 3U) : (rir >> 21U);
    frame->length = (uint8_t)(CAN_RDTR(base) & 0x0FU);
    if (frame->length > 8U) frame->length = 8U;
    for (uint32_t i = 0; i < 4U; ++i) {
        frame->data[i] = (uint8_t)(dlr >> (i * 8U));
        frame->data[i + 4U] = (uint8_t)(dhr >> (i * 8U));
    }
    CAN_RF0R(base) |= (1UL << 5); /* release FIFO 0 */
    return true;
}

bool can_transmit(const can_frame_t *frame)
{
    if ((frame == 0) || (frame->bus > CAN_BUS_2) || (frame->length > 8U) ||
        (frame->identifier > (frame->extended ? 0x1FFFFFFFUL : 0x7FFUL))) {
        return false;
    }
    const uint32_t base = frame->bus == CAN_BUS_1 ? CAN1_BASE : CAN2_BASE;
    const uint32_t tsr = CAN_TSR(base);
    uint32_t mailbox = (tsr & (1UL << 26)) ? 0U : ((tsr & (1UL << 27)) ? 1U : ((tsr & (1UL << 28)) ? 2U : 3U));
    if (mailbox == 3U) return false;
    const uint32_t offset = 0x180UL + mailbox * 0x10UL;
    *(volatile uint32_t *)(base + offset + 0x04UL) = frame->length;
    uint32_t low = 0U, high = 0U;
    for (uint32_t i = 0U; i < frame->length; ++i) {
        if (i < 4U) {
            low |= (uint32_t)frame->data[i] << (i * 8U);
        } else {
            high |= (uint32_t)frame->data[i] << ((i - 4U) * 8U);
        }
    }
    *(volatile uint32_t *)(base + offset + 0x08UL) = low;
    *(volatile uint32_t *)(base + offset + 0x0CUL) = high;
    *(volatile uint32_t *)(base + offset + 0x00UL) = frame->extended
        ? ((frame->identifier << 3U) | (1UL << 2U) | 1UL)
        : ((frame->identifier << 21U) | 1UL);
    return true;
}
