#include "bsp.h"

void lv_ex_bar_1(void)
{
	lv_obj_t * bar1 = lv_bar_create(lv_scr_act(), NULL);
	lv_obj_set_size(bar1, 200, 20);
	lv_obj_align(bar1, NULL, LV_ALIGN_CENTER, 0, 0);
	lv_bar_set_anim_time(bar1, 2000);
	lv_bar_set_value(bar1, 50, LV_ANIM_ON);
}
