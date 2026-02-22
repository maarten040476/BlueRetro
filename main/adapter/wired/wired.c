/*
 * Copyright (c) 2021-2024, Jacques Gagnon
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stddef.h>
#include "soc/gpio_struct.h"
#include "zephyr/types.h"
#include "tools/util.h"
#include "gc.h"
#include "wired.h"

int32_t wired_meta_init(struct wired_ctrl *ctrl_data) {
    gc_meta_init(ctrl_data);
    return 0;
}

void IRAM_ATTR wired_init_buffer(int32_t dev_mode, struct wired_data *wired_data) {
    gc_init_buffer(dev_mode, wired_data);
}

void wired_from_generic(int32_t dev_mode, struct wired_ctrl *ctrl_data, struct wired_data *wired_data) {
    gc_from_generic(dev_mode, ctrl_data, wired_data);
}

void wired_fb_to_generic(int32_t dev_mode, struct raw_fb *raw_fb_data, struct generic_fb *fb_data) {
    gc_fb_to_generic(dev_mode, raw_fb_data, fb_data);
}

void wired_para_turbo_mask_hdlr(void) {
    /* No parallel/SEA handling needed for GC-only build */
}

void IRAM_ATTR wired_gen_turbo_mask_btns16_pos(struct wired_data *wired_data, uint16_t *buttons, const uint32_t btns_mask[32]) {
    for (uint32_t i = 0; i < 32; i++) {
        uint8_t mask = wired_data->cnt_mask[i] >> 1;

        if (btns_mask[i] && mask) {
            if (wired_data->cnt_mask[i] & 1) {
                if (!(mask & wired_data->frame_cnt)) {
                    *buttons &= ~btns_mask[i];
                }
            }
            else {
                if (!((mask & wired_data->frame_cnt) == mask)) {
                    *buttons &= ~btns_mask[i];
                }
            }
        }
    }
}

void IRAM_ATTR wired_gen_turbo_mask_btns16_neg(struct wired_data *wired_data, uint16_t *buttons, const uint32_t btns_mask[32]) {
    for (uint32_t i = 0; i < 32; i++) {
        uint8_t mask = wired_data->cnt_mask[i] >> 1;

        if (btns_mask[i] && mask) {
            if (wired_data->cnt_mask[i] & 1) {
                if (!(mask & wired_data->frame_cnt)) {
                    *buttons |= btns_mask[i];
                }
            }
            else {
                if (!((mask & wired_data->frame_cnt) == mask)) {
                    *buttons |= btns_mask[i];
                }
            }
        }
    }
}

void IRAM_ATTR wired_gen_turbo_mask_btns32(struct wired_data *wired_data, uint32_t *buttons, const uint32_t (*btns_mask)[32],
                                            uint32_t bank_cnt) {
    for (uint32_t i = 0; i < 32; i++) {
        uint8_t mask = wired_data->cnt_mask[i] >> 1;

        if (mask) {
            for (uint32_t j = 0; j < bank_cnt; j++) {
                if (btns_mask[j][i]) {
                    if (wired_data->cnt_mask[i] & 1) {
                        if (!(mask & wired_data->frame_cnt)) {
                            buttons[j] |= btns_mask[j][i];
                        }
                    }
                    else {
                        if (!((mask & wired_data->frame_cnt) == mask)) {
                            buttons[j] |= btns_mask[j][i];
                        }
                    }
                }
            }
        }
    }
}

void IRAM_ATTR wired_gen_turbo_mask_axes8(struct wired_data *wired_data, uint8_t *axes, uint32_t axes_cnt,
                                            const uint8_t axes_idx[6], const struct ctrl_meta *axes_meta) {
    for (uint32_t i = 0; i < axes_cnt; i++) {
        uint8_t btn_id = axis_to_btn_id(i);
        uint8_t mask = wired_data->cnt_mask[btn_id] >> 1;
        if (mask) {
            if (wired_data->cnt_mask[btn_id] & 1) {
                if (!(mask & wired_data->frame_cnt)) {
                    axes[axes_idx[i]] = axes_meta[i].neutral;
                }
            }
            else {
                if (!((mask & wired_data->frame_cnt) == mask)) {
                    axes[axes_idx[i]] = axes_meta[i].neutral;
                }
            }
        }
    }
}
