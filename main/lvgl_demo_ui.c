/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "lvgl.h"

lv_obj_t* example_lvgl_demo_ui(lv_disp_t* disp, char* msg, lv_obj_t* old)
{
    if(old != NULL) lv_obj_clean(old);

    lv_obj_t* scr = lv_disp_get_scr_act(disp); //Get the current screen
    lv_obj_t* label = lv_label_create(scr); //Create a label on the currently active screen
    lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL_CIRCULAR); /* Circular scroll */
    lv_label_set_text(label, msg);
    
    /* Size of the screen (if you use rotation 90 or 270, please set disp->driver->ver_res) */
    lv_obj_set_width(label, disp->driver->hor_res);
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 0);
    return label;
}
