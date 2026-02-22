/*
 * Copyright (c) 2019-2024, Jacques Gagnon
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include "soc/io_mux_reg.h"
#include "soc/gpio_pins.h"
#include <hal/clk_gate_ll.h>
#include <soc/rmt_struct.h>
#include <hal/rmt_types.h>
#include <hal/rmt_ll.h>
#include <esp32/rom/ets_sys.h>
#include "zephyr/atomic.h"
#include "zephyr/types.h"
#include "tools/util.h"
#include "adapter/adapter.h"
#include "adapter/config.h"
#include "adapter/wired/gc.h"
#include "system/gpio.h"
#include "system/intr.h"
#include "nsi.h"

#define BIT_ZERO 0x80020006
#define BIT_ONE  0x80060002
#define BIT_ONE_MASK 0x7FFC0000
#define STOP_BIT_2US 0x80000004

#define GC_BIT_PERIOD_TICKS 10

#define GAME_ID_CMD 0x1D

#define RMT_MEM_ITEM_NUM SOC_RMT_MEM_WORDS_PER_CHANNEL

typedef struct {
    struct {
        rmt_symbol_word_t data32[SOC_RMT_MEM_WORDS_PER_CHANNEL];
    } chan[SOC_RMT_CHANNELS_PER_GROUP];
} rmt_mem_t;

extern rmt_mem_t RMTMEM;

static const uint8_t gpio_pin[4] = {
    19, 5, 26, 27
};

static const uint8_t rmt_ch[4][2] = {
    {0, 0},
    {1, 2},
    {2, 4},
    {3, 6},
};

static const uint8_t nsi_crc_table[256] = {
    0x8F, 0x85, 0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01, 0xC2, 0x61, 0xF2, 0x79, 0xFE, 0x7F,
    0xFD, 0xBC, 0x5E, 0x2F, 0xD5, 0xA8, 0x54, 0x2A, 0x15, 0xC8, 0x64, 0x32, 0x19, 0xCE, 0x67, 0xF1,
    0xBA, 0x5D, 0xEC, 0x76, 0x3B, 0xDF, 0xAD, 0x94, 0x4A, 0x25, 0xD0, 0x68, 0x34, 0x1A, 0x0D, 0xC4,
    0x62, 0x31, 0xDA, 0x6D, 0xF4, 0x7A, 0x3D, 0xDC, 0x6E, 0x37, 0xD9, 0xAE, 0x57, 0xE9, 0xB6, 0x5B,
    0xEF, 0xB5, 0x98, 0x4C, 0x26, 0x13, 0xCB, 0xA7, 0x91, 0x8A, 0x45, 0xE0, 0x70, 0x38, 0x1C, 0x0E,
    0x07, 0xC1, 0xA2, 0x51, 0xEA, 0x75, 0xF8, 0x7C, 0x3E, 0x1F, 0xCD, 0xA4, 0x52, 0x29, 0xD6, 0x6B,
    0xF7, 0xB9, 0x9E, 0x4F, 0xE5, 0xB0, 0x58, 0x2C, 0x16, 0x0B, 0xC7, 0xA1, 0x92, 0x49, 0xE6, 0x73,
    0xFB, 0xBF, 0x9D, 0x8C, 0x46, 0x23, 0xD3, 0xAB, 0x97, 0x89, 0x86, 0x43, 0xE3, 0xB3, 0x9B, 0x8F,
    0x85, 0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01, 0xC2, 0x61, 0xF2, 0x79, 0xFE, 0x7F, 0xFD,
    0xBC, 0x5E, 0x2F, 0xD5, 0xA8, 0x54, 0x2A, 0x15, 0xC8, 0x64, 0x32, 0x19, 0xCE, 0x67, 0xF1, 0xBA,
    0x5D, 0xEC, 0x76, 0x3B, 0xDF, 0xAD, 0x94, 0x4A, 0x25, 0xD0, 0x68, 0x34, 0x1A, 0x0D, 0xC4, 0x62,
    0x31, 0xDA, 0x6D, 0xF4, 0x7A, 0x3D, 0xDC, 0x6E, 0x37, 0xD9, 0xAE, 0x57, 0xE9, 0xB6, 0x5B, 0xEF,
    0xB5, 0x98, 0x4C, 0x26, 0x13, 0xCB, 0xA7, 0x91, 0x8A, 0x45, 0xE0, 0x70, 0x38, 0x1C, 0x0E, 0x07,
    0xC1, 0xA2, 0x51, 0xEA, 0x75, 0xF8, 0x7C, 0x3E, 0x1F, 0xCD, 0xA4, 0x52, 0x29, 0xD6, 0x6B, 0xF7,
    0xB9, 0x9E, 0x4F, 0xE5, 0xB0, 0x58, 0x2C, 0x16, 0x0B, 0xC7, 0xA1, 0x92, 0x49, 0xE6, 0x73, 0xFB,
    0xBF, 0x9D, 0x8C, 0x46, 0x23, 0xD3, 0xAB, 0x97, 0x89, 0x86, 0x43, 0xE3, 0xB3, 0x9B, 0x8F, 0x85,
};

static const uint8_t gc_ident[3] = {0x09, 0x00, 0x20};
static const uint8_t gc_kb_ident[3] = {0x08, 0x20, 0x00};
static const uint8_t gc_neutral[] = {
    0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x20, 0x20, 0x00, 0x00
};
static volatile rmt_symbol_word_t *rmt_items = (volatile rmt_symbol_word_t *)RMTMEM.chan[0].data32;
static uint8_t buf[128] = {0};
static uint32_t *buf32 = (uint32_t *)buf;
static uint16_t *buf16 = (uint16_t *)buf;
static uint8_t last_rumble[4] = {0};
static uint32_t gc_l_trig_prev_state[4] = {0};
static uint32_t gc_r_trig_prev_state[4] = {0};

static uint16_t nsi_bytes_to_items_crc(uint32_t item, const uint8_t *data, uint32_t len, uint8_t *crc, uint32_t stop_bit) {
    const uint8_t *crc_table = nsi_crc_table;
    uint32_t bit_len = item + len * 8;
    volatile uint32_t *item_ptr = &rmt_items[item].val;

    *crc = 0xFF;
    for (; item < bit_len; ++data) {
        for (uint32_t mask = 0x80; mask; mask >>= 1) {
            if (*data & mask) {
                *crc ^= *crc_table;
                *item_ptr = BIT_ONE;
            }
            else {
                *item_ptr = BIT_ZERO;
            }
            ++crc_table;
            ++item_ptr;
            ++item;
        }
    }
    *item_ptr = stop_bit;
    return item;
}

static uint16_t nsi_bytes_to_items_xor(uint32_t item, const uint8_t *data, uint32_t len, uint8_t *xor, uint32_t stop_bit) {
    uint32_t bit_len = item + len * 8;
    volatile uint32_t *item_ptr = &rmt_items[item].val;

    *xor = 0x00;
    for (; item < bit_len; ++data) {
        for (uint32_t mask = 0x80; mask; mask >>= 1) {
            if (*data & mask) {
                *item_ptr = BIT_ONE;
            }
            else {
                *item_ptr = BIT_ZERO;
            }
            ++item_ptr;
            ++item;
        }
        *xor ^= *data;
    }
    *item_ptr = stop_bit;
    return item;
}

static uint16_t nsi_items_to_bytes(uint32_t item, uint8_t *data, uint32_t len) {
    uint32_t bit_len = item + len * 8;
    volatile uint32_t *item_ptr = &rmt_items[item].val;

    for (; item < bit_len; ++data) {
        do {
            if (*item_ptr & BIT_ONE_MASK) {
                *data = (*data << 1) + 1;
            }
            else {
                *data <<= 1;
            }
            ++item_ptr;
            ++item;
        } while ((item % 8));
    }
    return item;
}

static void nsi_game_id_cmd_hdlr(uint8_t channel, uint8_t port, uint16_t item) {
    struct raw_fb fb_data = {0};

    item = nsi_items_to_bytes(item, buf, 10);
    RMT.conf_ch[channel].conf1.mem_rd_rst = 1;
    RMT.conf_ch[channel].conf1.mem_rd_rst = 0;
    RMT.conf_ch[channel].conf1.mem_owner = RMT_LL_MEM_OWNER_HW;
    RMT.conf_ch[channel].conf1.rx_en = 1;

    fb_data.header.wired_id = channel;
    fb_data.header.type = FB_TYPE_GAME_ID;
    fb_data.header.data_len = 8;
    for (uint32_t i = 0; i < 8; ++i) {
        fb_data.data[i] = buf[i];
    }
    adapter_q_fb(&fb_data);
}

static void gc_kb_cmd_hdlr(uint8_t channel, uint8_t port, uint16_t item) {
    uint8_t crc;

    switch (buf[0]) {
        case GAME_ID_CMD:
            nsi_game_id_cmd_hdlr(channel, port, item);
            break;
        case 0x00:
        case 0xFF:
            nsi_bytes_to_items_crc(channel * RMT_MEM_ITEM_NUM, gc_kb_ident, sizeof(gc_kb_ident), &crc, STOP_BIT_2US);
            RMT.conf_ch[channel].conf1.tx_start = 1;
            break;
        case 0x54:
            item = nsi_bytes_to_items_xor(channel * RMT_MEM_ITEM_NUM, wired_adapter.data[port].output, 7, &crc, STOP_BIT_2US);
            buf[0] = crc;
            nsi_bytes_to_items_xor(item, buf, 1, &crc, STOP_BIT_2US);
            RMT.conf_ch[channel].conf1.tx_start = 1;
            ++wired_adapter.data[port].frame_cnt;
            break;
        default:
            RMT.conf_ch[channel].conf1.mem_rd_rst = 1;
            RMT.conf_ch[channel].conf1.mem_rd_rst = 0;
            RMT.conf_ch[channel].conf1.mem_owner = RMT_LL_MEM_OWNER_HW;
            RMT.conf_ch[channel].conf1.rx_en = 1;
            break;
    }
}

static void gc_pad_cmd_hdlr(uint8_t channel, uint8_t port, uint16_t item) {
    uint8_t crc;

    switch (buf[0]) {
        case GAME_ID_CMD:
            nsi_game_id_cmd_hdlr(channel, port, item);
            break;
        case 0x00:
        case 0xFF:
            nsi_bytes_to_items_crc(channel * RMT_MEM_ITEM_NUM, gc_ident, sizeof(gc_ident), &crc, STOP_BIT_2US);
            RMT.conf_ch[channel].conf1.tx_start = 1;
            break;
        case 0x40:
        case 0x43:
        {
            uint8_t len = 8;
            nsi_items_to_bytes(item, &buf[1], 2);
            buf16[2] = wired_adapter.data[port].output16[0] & wired_adapter.data[port].output_mask16[0];
            for (uint32_t i = 6; i < 12; ++i) {
                buf[i] = (wired_adapter.data[port].output_mask[i - 4]) ?
                    wired_adapter.data[port].output_mask[i - 4] : wired_adapter.data[port].output[i - 4];
            }
            uint8_t trig_l = buf[10];
            uint8_t trig_r = buf[11];

            if (gc_r_trig_prev_state[port] < 2) {
                buf[5] &= ~0x20;
            }
            if (gc_l_trig_prev_state[port] < 2) {
                buf[5] &= ~0x40;
            }

            if (buf[0] == 0x43) {
                len = 10;
                buf[12] = 0;
                buf[13] = 0;
            }
            else switch (buf[1]) {
                case 1:
                    buf[8] &= 0xF0;
                    buf[8] |= buf[9] >> 4;
                    buf[9] = buf[10];
                    buf[10] = buf[11];
                    buf[11] = 0x00;
                    break;
                case 2:
                    buf[8] &= 0xF0;
                    buf[8] |= buf[9] >> 4;
                    buf[9] &= 0xF0;
                    buf[9] |= buf[11] >> 4;
                    buf[10] = 0x00;
                    buf[11] = 0x00;
                    break;
                case 3:
                    break;
                case 4:
                    buf[10] = 0x00;
                    buf[11] = 0x00;
                    break;
                default:
                    buf[10] &= 0xF0;
                    buf[10] |= buf[11] >> 4;
                    buf[11] = 0x00;
                    break;
            }
            nsi_bytes_to_items_crc(channel * RMT_MEM_ITEM_NUM, &buf[4], len, &crc, STOP_BIT_2US);
            RMT.conf_ch[channel].conf1.tx_start = 1;

            if (trig_l > 0x30) {
                if (gc_l_trig_prev_state[port] < 2) {
                    gc_l_trig_prev_state[port]++;
                }
            }
            else {
                gc_l_trig_prev_state[port] = 0;
            }
            if (trig_r > 0x30) {
                if (gc_r_trig_prev_state[port] < 2) {
                    gc_r_trig_prev_state[port]++;
                }
            }
            else {
                gc_r_trig_prev_state[port] = 0;
            }

            if (config.out_cfg[port].acc_mode == ACC_RUMBLE) {
                struct raw_fb fb_data = {0};

                fb_data.data[0] = buf[2] & 0x01;
                if (last_rumble[port] != fb_data.data[0]) {
                    last_rumble[port] = fb_data.data[0];
                    fb_data.header.wired_id = port;
                    fb_data.header.type = FB_TYPE_RUMBLE;
                    fb_data.header.data_len = 1;
                    adapter_q_fb(&fb_data);
                }
            }

            ++wired_adapter.data[port].frame_cnt;
            gc_gen_turbo_mask(&wired_adapter.data[port]);
            break;
        }
        case 0x41:
        case 0x42:
            nsi_bytes_to_items_crc(channel * RMT_MEM_ITEM_NUM, gc_neutral, sizeof(gc_neutral), &crc, STOP_BIT_2US);
            RMT.conf_ch[channel].conf1.tx_start = 1;
            wired_adapter.data[port].output[0] &= ~0x20;
            break;
        default:
            RMT.conf_ch[channel].conf1.mem_rd_rst = 1;
            RMT.conf_ch[channel].conf1.mem_rd_rst = 0;
            RMT.conf_ch[channel].conf1.mem_owner = RMT_LL_MEM_OWNER_HW;
            RMT.conf_ch[channel].conf1.rx_en = 1;
            break;
    }
}

static unsigned gc_isr(unsigned cause) {
    const uint32_t intr_st = RMT.int_st.val;
    uint32_t status = intr_st;
    uint16_t item;
    uint8_t i, channel, port;

    while (status) {
        i = __builtin_ffs(status) - 1;
        status &= ~(1 << i);
        channel = i / 3;
        port = channel / 2;
        switch (i % 3) {
            case 0:
                RMT.conf_ch[channel].conf1.mem_rd_rst = 1;
                RMT.conf_ch[channel].conf1.mem_rd_rst = 0;
                RMT.conf_ch[channel].conf1.mem_owner = RMT_LL_MEM_OWNER_HW;
                RMT.conf_ch[channel].conf1.rx_en = 1;
                break;
            case 1:
                RMT.conf_ch[channel].conf1.rx_en = 0;
                RMT.conf_ch[channel].conf1.mem_owner = RMT_LL_MEM_OWNER_SW;
                RMT.conf_ch[channel].conf1.mem_wr_rst = 1;
                item = nsi_items_to_bytes(channel * RMT_MEM_ITEM_NUM, buf, 1);
                switch (config.out_cfg[port].dev_mode) {
                    case DEV_KB:
                        gc_kb_cmd_hdlr(channel, port, item);
                        break;
                    default:
                        gc_pad_cmd_hdlr(channel, port, item);
                        break;
                }
                break;
            case 2:
                ets_printf("ERR\n");
                RMT.int_ena.val &= (~(BIT(i)));
                break;
            default:
                break;
        }
    }
    RMT.int_clr.val = intr_st;
    return 0;
}

void nsi_init(uint32_t package) {
    periph_ll_enable_clk_clear_rst(PERIPH_RMT_MODULE);

    RMT.apb_conf.fifo_mask = 1;

    nsi_port_cfg(0xF);

    for (uint32_t i = 0; i < ARRAY_SIZE(gpio_pin); i++) {
        RMT.conf_ch[rmt_ch[i][1]].conf0.div_cnt = 40;
        RMT.conf_ch[rmt_ch[i][1]].conf1.mem_rd_rst = 1;
        RMT.conf_ch[rmt_ch[i][1]].conf1.mem_wr_rst = 1;
        RMT.conf_ch[rmt_ch[i][1]].conf1.tx_conti_mode = 0;
        RMT.conf_ch[rmt_ch[i][1]].conf0.mem_size = 8;
        RMT.conf_ch[rmt_ch[i][1]].conf1.mem_owner = RMT_LL_MEM_OWNER_SW;
        RMT.conf_ch[rmt_ch[i][1]].conf1.ref_always_on = 1;
        RMT.conf_ch[rmt_ch[i][1]].conf1.idle_out_en = 1;
        RMT.conf_ch[rmt_ch[i][1]].conf1.idle_out_lv = 1;
        RMT.conf_ch[rmt_ch[i][1]].conf0.carrier_en = 0;
        RMT.conf_ch[rmt_ch[i][1]].conf0.carrier_out_lv = 0;
        RMT.carrier_duty_ch[rmt_ch[i][1]].high = 0;
        RMT.carrier_duty_ch[rmt_ch[i][1]].low = 0;
        RMT.conf_ch[rmt_ch[i][1]].conf0.idle_thres = GC_BIT_PERIOD_TICKS;
        RMT.conf_ch[rmt_ch[i][1]].conf1.rx_filter_thres = 0;
        RMT.conf_ch[rmt_ch[i][1]].conf1.rx_filter_en = 0;

        rmt_ll_enable_interrupt(&RMT, RMT_LL_EVENT_TX_DONE(rmt_ch[i][1]), 1);
        rmt_ll_enable_interrupt(&RMT, RMT_LL_EVENT_RX_DONE(rmt_ch[i][1]), 1);
        rmt_ll_enable_interrupt(&RMT, RMT_LL_EVENT_RX_ERROR(rmt_ch[i][1]), 1);

        rmt_ll_rx_enable(&RMT, rmt_ch[i][1], 0);
        rmt_ll_rx_reset_pointer(&RMT, rmt_ch[i][1]);
        rmt_ll_clear_interrupt_status(&RMT, RMT_LL_EVENT_RX_DONE(rmt_ch[i][1]));
        rmt_ll_enable_interrupt(&RMT, RMT_LL_EVENT_RX_DONE(rmt_ch[i][1]), 1);
        rmt_ll_rx_enable(&RMT, rmt_ch[i][1], 1);
    }

    intexc_alloc_iram(ETS_RMT_INTR_SOURCE, 19, gc_isr);
}

void nsi_port_cfg(uint16_t mask) {
    for (uint32_t i = 0; i < ARRAY_SIZE(gpio_pin); i++) {
        if (mask & 0x1) {
            PIN_FUNC_SELECT(GPIO_PIN_MUX_REG_IRAM[gpio_pin[i]], PIN_FUNC_GPIO);
            gpio_set_direction_iram(gpio_pin[i], GPIO_MODE_INPUT_OUTPUT_OD);
            gpio_matrix_out(gpio_pin[i], RMT_SIG_OUT0_IDX + rmt_ch[i][1], 0, 0);
            gpio_matrix_in(gpio_pin[i], RMT_SIG_IN0_IDX + rmt_ch[i][1], 0);
        }
        else {
            gpio_reset_iram(gpio_pin[i]);
            gpio_matrix_in(GPIO_MATRIX_CONST_ONE_INPUT, RMT_SIG_IN0_IDX + rmt_ch[i][1], 0);
        }
        mask >>= 1;
    }
}
