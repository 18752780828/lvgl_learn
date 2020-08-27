#include "bsp.h"

static void my_event_cb(lv_obj_t * obj, lv_event_t event);
static lv_style_t style1;
static lv_style_t style2;

void lvgl_btn_test(void)
{
	lv_obj_t * scr = lv_obj_create(NULL, NULL);
	lv_style_init(&style1);	
	lv_style_set_bg_color(&style1, LV_STATE_DEFAULT, LV_COLOR_GRAY);
	lv_scr_load(scr);	
	lv_obj_add_style(scr, LV_OBJ_PART_MAIN, &style1);
	
	/*Create  button1*/
	lv_style_init(&style2);	
	lv_obj_t * btn1 = lv_btn_create(scr, NULL); /*Create a button on the screen*/
	lv_btn_set_fit(btn1, true); /*Enable to automatically set the size according to the content*/
	lv_obj_set_pos(btn1, 60, 40); /*Set the position of the button*/
	
	lv_style_set_border_color(&style2, LV_STATE_DEFAULT, LV_COLOR_RED);
	lv_style_set_bg_color(&style2, LV_STATE_DEFAULT, LV_COLOR_BLUE);
	lv_style_set_border_width(&style2, LV_STATE_DEFAULT, 5);
	lv_style_set_border_opa(&style2, LV_STATE_DEFAULT, LV_OPA_50);
	lv_style_set_border_side(&style2, LV_STATE_DEFAULT, LV_BORDER_SIDE_BOTTOM | LV_BORDER_SIDE_RIGHT);
	lv_obj_add_style(btn1, LV_BTN_PART_MAIN, &style2);

	/*Create button2*/
	lv_obj_t * btn2 = lv_btn_create(scr, NULL); /*Copy the first button*/
	lv_btn_set_fit(btn2, true); /*Enable to automatically set the size according to the content*/
	lv_obj_set_pos(btn2, 200, 40); /*Set the position of the button*/
	
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
			lv_style_set_bg_color(&style2, LV_BTN_PART_MAIN, LV_COLOR_ORANGE);
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
}
