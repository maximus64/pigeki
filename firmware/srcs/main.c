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
#include <math.h>
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

#define BTN_STAB_L_LED_GPIO 25
#define BTN_STAB_R_LED_GPIO 6

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

/* This value limit the brightness of the WAD button LED. Max: 11730 */
#define WAD_RGB_TOTAL_LIMIT (7000)

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

typedef union {
    uint32_t val;
    struct {
        uint8_t b;
        uint8_t r;
        uint8_t g;
        uint8_t pad;
    };
} rgb_color_t;

static rgb_color_t button_colors[8] = {0};
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
static uint8_t lever_pos, wad_l_pos, wad_r_pos;

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

static void rgb_set_color(int idx, const rgb_color_t val) {
    uint32_t save = spin_lock_blocking(rgb_spin_lock);
    button_colors[idx] = val;
    spin_unlock(rgb_spin_lock, save);
}

static inline uint32_t rgb_sum_color(rgb_color_t in) {
    return ((uint32_t)in.r * 16 + (uint32_t)in.g * 15 + (uint32_t)in.b * 16);
}

static void rgb_scale(rgb_color_t *c, float scale)
{
    c->r = (uint8_t)((float)c->r * scale);
    c->g = (uint8_t)((float)c->g * scale);
    c->b = (uint8_t)((float)c->b * scale);
}

static void rgb_generate_pattern(uint32_t *bit_pattern) {
    int i, j;
    rgb_color_t colors[8];
    uint32_t wad_sum = 0;

    uint32_t save = spin_lock_blocking(rgb_spin_lock);
    memcpy(&colors[0], &button_colors[0], sizeof(colors));
    spin_unlock(rgb_spin_lock, save);

    /*
     * Need to limit power for WAD button because we drawn too much power from
     * USB port
     */
    wad_sum += rgb_sum_color(colors[WAD_L_RGB_IDX]);
    wad_sum += rgb_sum_color(colors[WAD_R_RGB_IDX]);

    if (wad_sum > WAD_RGB_TOTAL_LIMIT) {
        const float scale = (float) WAD_RGB_TOTAL_LIMIT / (float) wad_sum;
        rgb_scale(&colors[WAD_L_RGB_IDX], scale);
        rgb_scale(&colors[WAD_R_RGB_IDX], scale);
    }

    for (i = 0; i < 24; i++) {
        for (j = 0; j < 8; j++) {
            if (colors[j].val & BIT(23 - i))
                bit_pattern[i] |= BIT(rgb_gpios[j]);
            else
                bit_pattern[i] &= ~BIT(rgb_gpios[j]);
        }
    }
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

static float snapCurve(float x)
{
  float y = 1.0 / (x + 1.0);
  y = (1.0 - y) * 2.0;
  if(y > 1.0) {
    return 1.0;
  }
  return y;
}

static void filter_scale_wad_lever (void)
{
#define WAD_L_IN_MIN 140
#define WAD_L_IN_MAX 190
#define WAD_R_IN_MIN 150
#define WAD_R_IN_MAX 189
#define WAD_OUT_MIN 0
#define WAD_OUT_MAX 255

#define LEVER_IN_MIN 53
#define LEVER_IN_MAX 182
#define LEVER_OUT_MIN 0
#define LEVER_OUT_MAX 255

#define LEVER_SNAP_MULTIPLIER 0.0005
#define WAD_SNAP_MULTIPLIER 0.005

/* Filter strength for VBUS ADC: scale 0 -> 1, 1.0 - no filter, */
#define VBUS_FILTER_STRENGTH 0.0001f
/* 8 bit ADC reference value when VBUS is at 5v */
#define VBUS_5V_REFERENCE_VALUE 197

    const float wad_l_in_range = (float)(WAD_L_IN_MAX - WAD_L_IN_MIN);
    const float wad_r_in_range = (float)(WAD_R_IN_MAX - WAD_R_IN_MIN);
    const float wad_out_range = (float)(WAD_OUT_MAX - WAD_OUT_MIN);

    const float wad_l_factor = wad_out_range / wad_l_in_range;
    const float wad_l_base = (float)(WAD_OUT_MIN) - (float)(WAD_L_IN_MIN) * wad_l_factor;

    const float wad_r_factor = wad_out_range / wad_r_in_range;
    const float wad_r_base = (float)(WAD_OUT_MIN) - (float)(WAD_R_IN_MIN) * wad_r_factor;

    const float lever_in_range = (float)(LEVER_IN_MAX - LEVER_IN_MIN);
    const float lever_out_range = (float)(LEVER_OUT_MAX - LEVER_OUT_MIN);

    const float lever_factor = lever_out_range / lever_in_range;
    const float lever_base = (float)(LEVER_OUT_MIN) - (float)(LEVER_IN_MIN) * lever_factor;

    uint8_t lever_in = adc_buf[0];
    uint8_t wad_l_in = ~adc_buf[1]; /* inverted */
    uint8_t wad_r_in = ~adc_buf[2]; /* inverted */
    uint8_t vbus_in  = adc_buf[3];

#ifdef BYPASS_ADC_FILTER
    lever_pos = lever_in;
    wad_l_pos = wad_l_in;
    wad_r_pos = wad_r_in;

    return;
#endif

    /* filter for VBUS */
    static float vbus_filter;

    float vbus_diff = fabsf(vbus_in - vbus_filter);
    float vbus_snap = snapCurve(vbus_diff * VBUS_FILTER_STRENGTH);
    vbus_filter += (vbus_in - vbus_filter) * vbus_snap;

    /* Calculate error from 5v */
    float vbus_error = VBUS_5V_REFERENCE_VALUE / vbus_filter;

    float lever_out, wad_l_out, wad_r_out;

    /*
     * Scale ADC input to expected range
     *
     * Since the lever hall effect sensor are reference of VBUS, we need to
     * compensate for voltage drop on VBUS line since LED can draw lot of
     * current and causing lever reading to be lower than normal.
     */
    lever_out = lever_base + ((float)lever_in * vbus_error) * lever_factor;
    wad_l_out = wad_l_base + (float)wad_l_in * wad_l_factor;
    wad_r_out = wad_r_base + (float)wad_r_in * wad_r_factor;


    /* filter noise */
    /* based on ResponsiveAnalogRead: https://github.com/dxinteractive/ResponsiveAnalogRead*/
    static float lever_smooth, wad_l_smooth, wad_r_smooth;

    /* get difference between new input value and current smooth value */
    float lever_diff = fabsf(lever_out - lever_smooth);
    float wad_l_diff = fabsf(wad_l_out - wad_l_smooth);
    float wad_r_diff = fabsf(wad_r_out - wad_r_smooth);

    /*
     * use a 'snap curve' function, where we pass in the diff (x) and get back a number from 0-1.
     * We want small values of x to result in an output close to zero, so when the smooth value is close to the input value
     * it'll smooth out noise aggressively by responding slowly to sudden changes.
     * We want a small increase in x to result in a much higher output value, so medium and large movements are snappy and responsive,
     * and aren't made sluggish by unnecessarily filtering out noise. A hyperbola (f(x) = 1/x) curve is used.
     * First x has an offset of 1 applied, so x = 0 now results in a value of 1 from the hyperbola function.
     * High values of x tend toward 0, but we want an output that begins at 0 and tends toward 1, so 1-y flips this up the right way.
     * Finally the result is multiplied by 2 and capped at a maximum of one, which means that at a certain point all larger movements are maximally snappy
     *
     * then multiply the input by SNAP_MULTIPLER so input values fit the snap curve better.
     */
    float lever_snap = snapCurve(lever_diff * LEVER_SNAP_MULTIPLIER);
    float wad_l_snap = snapCurve(wad_l_diff * WAD_SNAP_MULTIPLIER);
    float wad_r_snap = snapCurve(wad_r_diff * WAD_SNAP_MULTIPLIER);

    /* calculate the exponential moving average based on the snap */
    lever_smooth += (lever_out - lever_smooth) * lever_snap;
    wad_l_smooth += (wad_l_out - wad_l_smooth) * wad_l_snap;
    wad_r_smooth += (wad_r_out - wad_r_smooth) * wad_r_snap;

    // ensure output is in bounds
    if(lever_smooth < 0.0f)
        lever_pos = 0;
    else if(lever_smooth > 255.0f)
        lever_pos = 255;
    else
        lever_pos = (uint8_t)lever_smooth;

    if(wad_l_smooth < 0.0f)
        wad_l_pos = 0;
    else if(wad_l_smooth > 255.0f)
        wad_l_pos = 255;
    else
        wad_l_pos = (uint8_t)wad_l_smooth;

    if(wad_r_smooth < 0.0f)
        wad_r_pos = 0;
    else if(wad_r_smooth > 255.0f)
        wad_r_pos = 255;
    else
        wad_r_pos = (uint8_t)wad_r_smooth;
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
        filter_scale_wad_lever();

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
        report.x = lever_pos;
        report.y = wad_l_pos;
        report.z = wad_r_pos;

        tud_hid_n_report(0x00, 1, &report, sizeof(report));
    }
}

static void do_startup_led_animation(void) {
    /* default colors */
    static const rgb_color_t default_colors[8] = {
        { .r = 255 },
        { .g = 255 },
        { .b = 255 },
        { .r = 233, .g = 87, .b = 255 },
        { .r = 255 },
        { .g = 255 },
        { .b = 255 },
        { .r = 233, .g = 87, .b = 255 },
    };
    static const uint16_t stab_default = 0x4000;

    int i, j;
    const int steps = 1024;
    const float interval = 1.0f/steps;
    float scale = 0.0f;

    for (i = 0; i < steps; i++) {
        for (j = 0; j < 8; j++) {
            rgb_color_t c = default_colors[j];
            rgb_scale(&c, scale);
            rgb_set_color(j, c);
        }
        pwm_set_gpio_level(BTN_STAB_L_LED_GPIO, (uint16_t)(stab_default * scale));
        pwm_set_gpio_level(BTN_STAB_R_LED_GPIO, (uint16_t)(stab_default * scale));
        scale += interval;
        sleep_ms(3);
    }
}

int main() {
    int i;

    board_init();
    //stdio_init_all();
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

    /* check recovery button */
    if (!gpio_get(BTN_STAB_R_SW_GPIO)) {
        //printf("Go to bootrom USB boot\n");
        reset_usb_boot(0,0);
        panic("returned from USB boot??");
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

    /* turn off for now */
    pwm_set_gpio_level(BTN_STAB_L_LED_GPIO, 0);
    pwm_set_gpio_level(BTN_STAB_R_LED_GPIO, 0);

    /* initialize debounce state */
    debounced_state = sio_hw->gpio_in;

    multicore_launch_core1(core1_entry);

    do_startup_led_animation();

    while (true) {
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
    (void) itf;
    int i;

    typedef struct {
        struct {
            uint8_t r;
            uint8_t g;
            uint8_t b;
        } rgb[8];
        uint8_t stab_l;
        uint8_t stab_r;
    } set_led_report_t;

    if (report_id == 1 &&
        report_type == HID_REPORT_TYPE_OUTPUT &&
        bufsize == 26) {
        const set_led_report_t *report = (const set_led_report_t *) buffer;
        for (i = 0; i < 8; i++) {
            const rgb_color_t c = { .r = report->rgb[i].r,
                                .g = report->rgb[i].g,
                                .b = report->rgb[i].b};
            rgb_set_color(i, c);
        }
        pwm_set_gpio_level(BTN_STAB_L_LED_GPIO, report->stab_l << 8);
        pwm_set_gpio_level(BTN_STAB_R_LED_GPIO, report->stab_r << 8);
    }
}
