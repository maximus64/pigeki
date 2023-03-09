#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"

#define ARRAY_SIZE(x) (sizeof(x)/sizeof(*x))

#define BTN_L1_RGB_GPIO 22
#define BTN_L2_RGB_GPIO 20
#define BTN_L3_RGB_GPIO 18
#define WAD_L_RGB_GPIO 17

#define BTN_R1_RGB_GPIO 8
#define BTN_R2_RGB_GPIO 11
#define BTN_R3_RGB_GPIO 13
#define WAD_R_RGB_GPIO 14

#define BIT(x) (1UL << (x))

#define RGB_BIT_MASK ( BIT(BTN_L1_RGB_GPIO) | \
                       BIT(BTN_L2_RGB_GPIO) | \
                       BIT(BTN_L3_RGB_GPIO) | \
                       BIT(WAD_L_RGB_GPIO)  | \
                       BIT(BTN_R1_RGB_GPIO) | \
                       BIT(BTN_R2_RGB_GPIO) | \
                       BIT(BTN_R3_RGB_GPIO) | \
                       BIT(WAD_R_RGB_GPIO) )

#define LED_CHAIN_LENGTH 9

static const int rgb_gpios[8] = {
    BTN_L1_RGB_GPIO,
    BTN_L2_RGB_GPIO,
    BTN_L3_RGB_GPIO,
    WAD_L_RGB_GPIO,
    BTN_R1_RGB_GPIO,
    BTN_R2_RGB_GPIO,
    BTN_R3_RGB_GPIO,
    WAD_R_RGB_GPIO
};

static uint32_t button_colors[8] = {
    0xff0000,
    0x00ff00,
    0x0000ff,
    0xffffff,
    0xff00ff,
    0x00ffff,
    0xffff00,
    0x888888
};

/*
  ASM bitbang for WS2812B RGB LED
  Datasheet: https://cdn-shop.adafruit.com/datasheets/WS2812B.pdf

 0 Code:   T0H T0L
 1 Code:   T1H T1L
 RET Code: Treset

 T0H: 0.4us T0L: 0.85us +/- 150ns
        HHHHLLLLLLLL
 T1H: 0.8us T1L: 0.45us +/- 150ns
        HHHHHHHHLLLL
 Tres: >50us

 common: H X L
         1 2 3
 T1: 0.4us
 T2: 0.4us
 T3: 0.45us

pop bit from u32 input
clear_mask = (~value) & ALL_RGB_MASK
1. Set all RGB GPIO to HIGH
2. wait 0.4us
3. write clear mask for low GPIO group
5. wait 0.4us
6. Set all RGB GPIO to LOW
7. wait 0.45 us

*/

/* currently take ~280us*/
void __no_inline_not_in_flash_func(rgb_shift)(const uint32_t *bit_pattern) {
    static const volatile uint32_t* sio_base = (volatile uint32_t*)SIO_BASE;
    static const uint32_t rgb_mask = RGB_BIT_MASK;

    int i, j;

    for (j = 0; j < LED_CHAIN_LENGTH; j++) {
        for (i = 0; i < 24; i++) {
            uint32_t clear_mask = (~(bit_pattern[i])) & rgb_mask;

            asm volatile (
                "str %[mask], [%[sio_base], %[set_off]]\n" /* Set all RGB GPIO to HIGH  */
                "mov r3, %[t1]\n" /* wait 0.4us */
                "1: sub r3, #1\n"
                "bne 1b\n"
                "nop\n"
                "str %[clr_mask], [%[sio_base], %[clr_off]]\n" /* write clear mask for low GPIO group */
                "mov r3, %[t1]\n" /* wait 0.4us */
                "1: sub r3, #1\n"
                "bne 1b\n"
                "nop\n"
                "str %[mask], [%[sio_base], %[clr_off]]\n" /* Set all RGB GPIO to LOW  */
                "mov r3, %[t2]\n" /* wait 0.45us */
                "1: sub r3, #1\n"
                "bne 1b\n"
            :
            : [mask] "l" (rgb_mask),
            [clr_mask] "l" (clear_mask),
            [sio_base] "l" (sio_base),
            [set_off] "i" (SIO_GPIO_OUT_SET_OFFSET),
            [clr_off] "i" (SIO_GPIO_OUT_CLR_OFFSET),
            [t1] "i" (16),
            [t2] "i" (18)
            : "r3", "cc"
            );
        };
    };
}

void rgb_set_color(uint32_t *bit_pattern)
{
    int i, j;
    for (i = 0; i < 24; i++) {
        for (j = 0; j < 8; j++) {
            if (button_colors[j] & BIT(i))
                bit_pattern[i] |= BIT(rgb_gpios[j]);
            else
                bit_pattern[i] &= ~BIT(rgb_gpios[j]);
        }
    }
}

void core1_entry() {
    int i;
    uint32_t rgb_bit_pattern[24] = {0};
    printf("Hello from core 1\n");

    while (1) {
        printf("Go\n");
        rgb_set_color(rgb_bit_pattern);
        rgb_shift(rgb_bit_pattern);
        sleep_ms(1000);
    }
}

int main() {
    int i;

    stdio_init_all();

    /* set all RGB gpio to output */
    for (i = 0; i < ARRAY_SIZE(rgb_gpios); i++) {
        gpio_init(rgb_gpios[i]);
        gpio_put(rgb_gpios[i], 0); // drive low
        gpio_set_dir(rgb_gpios[i], GPIO_OUT);
    }

    multicore_launch_core1(core1_entry);

    printf("Pigeki hello!\n");

    while (true)
        continue;


    return 0;
}