/*
 * Copyright (c) 2023 Khoa Hoang <admin@khoahoang.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the “Software”),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/bootrom.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/dma.h"

#include "bsp/board.h"
#include "tusb.h"

#define ARRAY_SIZE(x) (sizeof(x)/sizeof(*x))
#define BIT(x) (1UL << (x))

/* GPIO pins definition */
#define BTN_L1_RGB_GPIO 22
#define BTN_L2_RGB_GPIO 20
#define BTN_L3_RGB_GPIO 18
#define WAD_L_RGB_GPIO 17

#define BTN_R1_RGB_GPIO 8
#define BTN_R2_RGB_GPIO 11
#define BTN_R3_RGB_GPIO 13
#define WAD_R_RGB_GPIO 14

#define BTN_STAB_L_LED_GPIO 6
#define BTN_STAB_R_LED_GPIO 25

#define BTN_L1_SW_GPIO 23
#define BTN_L2_SW_GPIO 21
#define BTN_L3_SW_GPIO 19
#define BTN_STAB_L_SW_GPIO 24

#define BTN_R1_SW_GPIO 9
#define BTN_R2_SW_GPIO 10
#define BTN_R3_SW_GPIO 12
#define BTN_STAB_R_SW_GPIO 7

#define LEVER_ADC_GPIO 26
#define WAD_L_ADC_GPIO 27
#define WAD_R_ADC_GPIO 28
#define VBUS_ADC_GPIO 29 /* Measure VBUS so we can correct lever input */


#define RGB_BIT_MASK ( BIT(BTN_L1_RGB_GPIO) | \
                       BIT(BTN_L2_RGB_GPIO) | \
                       BIT(BTN_L3_RGB_GPIO) | \
                       BIT(WAD_L_RGB_GPIO)  | \
                       BIT(BTN_R1_RGB_GPIO) | \
                       BIT(BTN_R2_RGB_GPIO) | \
                       BIT(BTN_R3_RGB_GPIO) | \
                       BIT(WAD_R_RGB_GPIO) )

/* Longest LED chain length - WAD have 9x3 in parallel */
#define LED_CHAIN_LENGTH 9

#define BTN_L1_RGB_IDX 0
#define BTN_L2_RGB_IDX 1
#define BTN_L3_RGB_IDX 2
#define WAD_L_RGB_IDX  3
#define BTN_R1_RGB_IDX 4
#define BTN_R2_RGB_IDX 5
#define BTN_R3_RGB_IDX 6
#define WAD_R_RGB_IDX  7

static const uint8_t rgb_gpios[8] = {
    BTN_L1_RGB_GPIO,
    BTN_L2_RGB_GPIO,
    BTN_L3_RGB_GPIO,
    WAD_L_RGB_GPIO,
    BTN_R1_RGB_GPIO,
    BTN_R2_RGB_GPIO,
    BTN_R3_RGB_GPIO,
    WAD_R_RGB_GPIO
};

#define COLOR_BLUE  0xff0000
#define COLOR_RED   0x00ff00
#define COLOR_GREEN 0x0000ff

static uint32_t button_colors[8] = {
    0xff0000, //Blue
    0x00ff00, //Red
    0x0000ff, //Green
    0xffffff,
    0xff00ff,
    0x00ffff,
    0xffff00,
    0x888888
};

static spin_lock_t *rgb_spin_lock;

static const uint8_t sw_gpios[] = {
    BTN_L1_SW_GPIO,
    BTN_L2_SW_GPIO,
    BTN_L3_SW_GPIO,
    BTN_STAB_L_SW_GPIO,
    BTN_R1_SW_GPIO,
    BTN_R2_SW_GPIO,
    BTN_R3_SW_GPIO,
    BTN_STAB_R_SW_GPIO,
};

static uint8_t adc_buf[4];

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

static void rgb_set_color(int idx, uint32_t val) {
    uint32_t save = spin_lock_blocking(rgb_spin_lock);
    button_colors[idx] = val;
    spin_unlock(rgb_spin_lock, save);
}

static void rgb_generate_pattern(uint32_t *bit_pattern) {
    int i, j;

    uint32_t save = spin_lock_blocking(rgb_spin_lock);
    for (i = 0; i < 24; i++) {
        for (j = 0; j < 8; j++) {
            if (button_colors[j] & BIT(i))
                bit_pattern[i] |= BIT(rgb_gpios[j]);
            else
                bit_pattern[i] &= ~BIT(rgb_gpios[j]);
        }
    }
    spin_unlock(rgb_spin_lock, save);
}


static volatile uint32_t debounced_state;

static void button_debounce(void) {
    /* four bits vertical counter debounce */
    static uint32_t cnt0, cnt1, cnt2, cnt3;
    uint32_t delta;
    uint32_t sample = sio_hw->gpio_in;

    delta = sample ^ debounced_state;
    cnt3 = (cnt3 ^ (cnt2 & cnt1 & cnt0)) & (delta & sample);
    cnt2 = (cnt2 ^ (cnt1 & cnt0)) & (delta & sample);
    cnt1 = (cnt1 ^ cnt0) & (delta & sample);
    cnt0 = ~cnt0 & (delta & sample);
    debounced_state ^= (delta & ~(cnt0 | cnt1 | cnt2 | cnt3));
}

static inline bool button_get(int gpio) {
    return !(debounced_state & BIT(gpio));
}

void core1_entry() {
    int i;
    static uint32_t rgb_bit_pattern[24] = {0};
    uint32_t ts;

    rgb_generate_pattern(rgb_bit_pattern);

    while (1) {
        rgb_shift(rgb_bit_pattern);

        ts = timer_hw->timerawl;
        rgb_generate_pattern(rgb_bit_pattern);
        button_debounce();

        /* Make sure Tres >50us is meet */
        while (timer_hw->timerawl - ts < 50) {
            tight_loop_contents();
        }
    }
}

static void sendReportData(void) {
    struct report {
        uint8_t buttons;
        uint8_t x;
        uint8_t y;
        uint8_t z;
    } report;

    // Poll every 1ms
    const uint32_t interval_ms = 1;
    static uint32_t start_ms = 0;

    if (board_millis() - start_ms < interval_ms) return;  // not enough time
    start_ms += interval_ms;

    if (tud_hid_ready()) {
        report.buttons = button_get(BTN_L1_SW_GPIO) |
                         (button_get(BTN_L2_SW_GPIO) << 1) |
                         (button_get(BTN_L3_SW_GPIO) << 2) |
                         (button_get(BTN_STAB_L_SW_GPIO) << 3) |
                         (button_get(BTN_R1_SW_GPIO) << 4) |
                         (button_get(BTN_R2_SW_GPIO) << 5) |
                         (button_get(BTN_R3_SW_GPIO) << 6) |
                         (button_get(BTN_STAB_R_SW_GPIO) << 7);
        report.x = adc_buf[0];
        report.y = adc_buf[1];
        report.z = adc_buf[2];

        tud_hid_n_report(0x00, 1, &report, sizeof(report));
    }
}

int main() {
    int i;

    board_init();
    stdio_init_all();
    tusb_init();

    rgb_spin_lock = spin_lock_instance(next_striped_spin_lock_num());

    /* set all RGB gpio to output */
    for (i = 0; i < ARRAY_SIZE(rgb_gpios); i++) {
        gpio_init(rgb_gpios[i]);
        gpio_put(rgb_gpios[i], 0); // drive low
        gpio_set_dir(rgb_gpios[i], GPIO_OUT);
        gpio_set_slew_rate(rgb_gpios[i], GPIO_SLEW_RATE_SLOW);
        gpio_set_drive_strength(rgb_gpios[i], GPIO_DRIVE_STRENGTH_2MA);
    }

    /* set all switch gpio to input */
    for (i = 0; i < ARRAY_SIZE(sw_gpios); i++) {
        gpio_init(sw_gpios[i]);
        gpio_set_dir(sw_gpios[i], GPIO_IN);
        gpio_pull_up(sw_gpios[i]);
    }

    {
        uint dma_ctrl_channel, dma_data_channel;
        dma_channel_config ctrl_cfg, data_cfg;
        static const uint8_t* write_addr[] = { &adc_buf[0] };

        /* setup adc */
        adc_init();
        adc_gpio_init(WAD_L_ADC_GPIO);
        adc_gpio_init(WAD_R_ADC_GPIO);
        adc_gpio_init(LEVER_ADC_GPIO);
        adc_gpio_init(VBUS_ADC_GPIO);

        adc_select_input(0);
        adc_set_round_robin(0xf);
        adc_fifo_setup(true, true, 1, false, true);
        adc_set_clkdiv(250.f);

        dma_ctrl_channel = dma_claim_unused_channel(true);
        dma_data_channel = dma_claim_unused_channel(true);

        ctrl_cfg = dma_channel_get_default_config(dma_ctrl_channel);
        channel_config_set_transfer_data_size(&ctrl_cfg, DMA_SIZE_32);
        channel_config_set_read_increment(&ctrl_cfg, false);
        channel_config_set_write_increment(&ctrl_cfg, false);
        dma_channel_configure(dma_ctrl_channel, &ctrl_cfg,
            &dma_hw->ch[dma_data_channel].al2_write_addr_trig,
            &write_addr, 1, false);

        data_cfg = dma_channel_get_default_config(dma_data_channel);
        channel_config_set_transfer_data_size(&data_cfg, DMA_SIZE_8);
        channel_config_set_read_increment(&data_cfg, false);
        channel_config_set_write_increment(&data_cfg, true);
        channel_config_set_chain_to(&data_cfg, dma_ctrl_channel);
        channel_config_set_dreq(&data_cfg, DREQ_ADC);
        dma_channel_configure(dma_data_channel, &data_cfg, adc_buf, &adc_hw->fifo, 4, true);

        /* start ADC capture */
        adc_run(true);
    }

    /* setup PWM for stab button */
    gpio_set_function(BTN_STAB_L_LED_GPIO, GPIO_FUNC_PWM);
    gpio_set_function(BTN_STAB_R_LED_GPIO, GPIO_FUNC_PWM);
    gpio_set_slew_rate(BTN_STAB_L_LED_GPIO, GPIO_SLEW_RATE_SLOW);
    gpio_set_slew_rate(BTN_STAB_R_LED_GPIO, GPIO_SLEW_RATE_SLOW);
    gpio_set_drive_strength(BTN_STAB_L_LED_GPIO, GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength(BTN_STAB_R_LED_GPIO, GPIO_DRIVE_STRENGTH_2MA);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 4.f);
    pwm_init(pwm_gpio_to_slice_num(BTN_STAB_L_LED_GPIO), &config, true);
    pwm_init(pwm_gpio_to_slice_num(BTN_STAB_R_LED_GPIO), &config, true);

    pwm_set_gpio_level(BTN_STAB_L_LED_GPIO, 0x8000); /* default to 50% duty */
    pwm_set_gpio_level(BTN_STAB_R_LED_GPIO, 0x8000); /* default to 50% duty */

    /* initialize debounce state */
    debounced_state = sio_hw->gpio_in;

    /* For debug */
    gpio_init(2);
    gpio_init(3);
    gpio_set_dir(2, GPIO_OUT);
    gpio_set_dir(3, GPIO_OUT);

    /* check recovery button */
    /* TODO: Make this harder to accidently press */
    if (!gpio_get(BTN_STAB_R_SW_GPIO)) {
        printf("Go to bootrom USB boot\n");
        reset_usb_boot(0,0);
        panic("returned from USB boot??");
    }

    multicore_launch_core1(core1_entry);

    printf("Pigeki hello!\n");

    // rgb_set_color(BTN_L1_RGB_IDX, COLOR_BLUE);
    // sleep_ms(1000);
    // rgb_set_color(BTN_L1_RGB_IDX, COLOR_RED);
    // sleep_ms(1000);
    // rgb_set_color(BTN_L1_RGB_IDX, COLOR_GREEN);
    // sleep_ms(1000);
    // rgb_set_color(BTN_L1_RGB_IDX, COLOR_BLUE | COLOR_RED);
    // sleep_ms(1000);
    // rgb_set_color(BTN_L1_RGB_IDX, COLOR_GREEN | COLOR_RED);
    // sleep_ms(3000);


    while (true) {
        // gpio_put(2, gpio_get(BTN_L1_SW_GPIO));
        // gpio_put(3, !!(debounced_state & BIT(BTN_L1_SW_GPIO)));
        // printf("lever: %d wadl: %d wadr: %d vbus: %d\n",
        //        adc_buf[0], adc_buf[1], adc_buf[2], adc_buf[3]);
        tud_task();
        sendReportData();
    }


    return 0;
}


// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t itf,
                               uint8_t report_id,
                               hid_report_type_t report_type,
                               uint8_t* buffer,
                               uint16_t reqlen)
{
  // TODO not Implemented
  (void) itf;
  (void) report_id;
  (void) report_type;
  (void) buffer;
  (void) reqlen;

  return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t itf,
                           uint8_t report_id,
                           hid_report_type_t report_type,
                           uint8_t const* buffer,
                           uint16_t bufsize)
{
  // TODO set RGB LEDs
  (void) itf;
  (void) report_id;
  (void) report_type;
  (void) buffer;
  (void) bufsize;
}
