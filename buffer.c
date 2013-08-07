/*
 * stack.c
 *
 *  Created on: 17 Jul 2013
 *      Author: Rene
 *
 * ----------------------------------------
 * |bottom    |      |      |       | top |
 * ----------------------------------------
 * push = new element is new top
 * pop = remove bottom
 */
// #include "buffer.h"

typedef struct element element_t;

struct element {
	int data;
	element_t * prev;
	element_t * next;
};
/* elemented_list_t contains a elemented list. */

typedef struct element_list {
	int count;
	element_t * top;
	element_t * bottom;
} element_list_t;

element_list_t list;

void buffer_init() {

	list.top = list.bottom = 0;
	list.count = 0;
}

int buffer_pop() {
	element_t * bottom;
	int result = -1;
	bottom = list.bottom;

	if (/*bottom && */list.count > 0) {
		result = bottom->data;
		list.bottom = bottom->next;
		free(bottom);
		list.count--;
	}
	return result;
}

int buffer_push(int input_data) {
	element_t * element;
	int data = input_data;

	if(list.count >= 50){
		buffer_pop();
	} 

	/* calloc sets the "next" field to zero. */
	element = (element_t *) malloc(sizeof(element_t));
	element->data = data;
	if (!element) {
		return 0;
	}
	if (!list.bottom) {
		list.bottom = element;
	}
	element->next = 0;
	element->prev = list.top;
	if (list.top) {
		list.top->next = element;
	}
	list.top = element;
	list.count++;
	return 1;
}

int buffer_size() {
	return list.count;
}

void buffer_reset() {
	buffer_init();
}
