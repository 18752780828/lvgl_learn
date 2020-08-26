#include "bsp.h"


static void my_event_cb(lv_obj_t * obj, lv_event_t event);
<<<<<<< HEAD
=======
static lv_style_t style1;
static lv_style_t style2;
>>>>>>> 008184c... touch done

void lvgl_obj_test(void)
{
	lv_obj_t * scr = lv_obj_create(NULL, NULL);
<<<<<<< HEAD
	lv_scr_load(scr);	
	
	/*Create 2 buttons*/
=======
	lv_style_init(&style1);	
	lv_style_set_bg_color(&style1, LV_STATE_DEFAULT, LV_COLOR_GRAY);
	lv_scr_load(scr);	
	lv_obj_add_style(scr, LV_OBJ_PART_MAIN, &style1);
	
//	lv_style_set_bg_color(&style1, LV_STATE_FOCUSED, LV_COLOR_RED);
	/*Create 2 buttons*/
	lv_style_init(&style2);	
>>>>>>> 008184c... touch done
	lv_obj_t * btn1 = lv_btn_create(scr, NULL); /*Create a button on the screen*/
	lv_btn_set_fit(btn1, true); /*Enable to automatically set the size according to the content*/
	lv_obj_set_pos(btn1, 60, 40); /*Set the position of the button*/
	
<<<<<<< HEAD
	lv_obj_t * btn2 = lv_btn_create(scr, btn1); /*Copy the first button*/
=======
	lv_style_set_bg_color(&style2, LV_BTN_PART_MAIN, LV_COLOR_BLUE);
	lv_obj_add_style(btn1, LV_BTN_PART_MAIN, &style2);
	
	lv_obj_refresh_style(btn1, LV_BTN_PART_MAIN, LV_STYLE_PROP_ALL);
	lv_obj_refresh_style(scr, LV_OBJ_PART_MAIN, LV_STYLE_PROP_ALL);

	lv_obj_t * btn2 = lv_btn_create(scr, NULL); /*Copy the first button*/
	lv_btn_set_fit(btn2, true); /*Enable to automatically set the size according to the content*/
>>>>>>> 008184c... touch done
	lv_obj_set_pos(btn2, 180, 80); /*Set the position of the button*/
	
	/*Add labels to the buttons*/
	lv_obj_t * label1 = lv_label_create(btn1, NULL); /*Create a label on the first button*/
	lv_label_set_text(label1, "exit"); /*Set the text of the label*/
	
	lv_obj_t * label2 = lv_label_create(btn2, NULL); /*Create a label on the second button*/
	lv_label_set_text(label2, "enter"); /*Set the text of the label*/
	
	lv_obj_set_event_cb(btn1, my_event_cb); /*Assign an event callback*/
}

static void my_event_cb(lv_obj_t * obj, lv_event_t event)
{
	switch(event) 
	{
		case LV_EVENT_PRESSED:
			printf("Pressed\n");
<<<<<<< HEAD
=======
			lv_style_set_bg_color(&style2, LV_BTN_PART_MAIN, LV_COLOR_ORANGE);
>>>>>>> 008184c... touch done
			break;
		case LV_EVENT_SHORT_CLICKED:
			printf("Short clicked\n");
			break;
		case LV_EVENT_CLICKED:
			printf("Clicked\n");
			break;
		case LV_EVENT_LONG_PRESSED:
			printf("Long press\n");
			break;
		case LV_EVENT_LONG_PRESSED_REPEAT:
			printf("Long press repeat\n");
			break;
		case LV_EVENT_RELEASED:
			printf("Released\n");
			break;
	}
	/*Etc.*/
<<<<<<< HEAD
}
=======
}
>>>>>>> 008184c... touch done
