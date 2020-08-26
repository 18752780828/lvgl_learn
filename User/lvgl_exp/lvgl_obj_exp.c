#include "bsp.h"


static void my_event_cb(lv_obj_t * obj, lv_event_t event);

void lvgl_obj_test(void)
{
	lv_obj_t * scr = lv_obj_create(NULL, NULL);
	lv_scr_load(scr);	
	
	/*Create 2 buttons*/
	lv_obj_t * btn1 = lv_btn_create(scr, NULL); /*Create a button on the screen*/
	lv_btn_set_fit(btn1, true); /*Enable to automatically set the size according to the content*/
	lv_obj_set_pos(btn1, 60, 40); /*Set the position of the button*/
	
	lv_obj_t * btn2 = lv_btn_create(scr, btn1); /*Copy the first button*/
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