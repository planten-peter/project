/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "lvgl.h"

lv_obj_t* example_lvgl_demo_ui(lv_disp_t* disp, char* msg, lv_obj_t* old)
{
    lv_obj_t* scr = lv_disp_get_scr_act(disp); //Get the current screen
    if(old == NULL){
        old = lv_label_create(scr); //Create a label on the currently active screen
    }
    lv_label_set_long_mode(old, LV_LABEL_LONG_SCROLL_CIRCULAR); /* Circular scroll */
    lv_label_set_text(old, msg);
    
    /* Size of the screen (if you use rotation 90 or 270, please set disp->driver->ver_res) */
    lv_obj_set_width(old, disp->driver->hor_res);
    lv_obj_align(old, LV_ALIGN_TOP_MID, 0, 0);
    return old;
}
