/*
 * stack.h
 *
 *  Created on: 17 Jul 2013
 *      Author: Rene
 */

#ifndef BUFFER_H_
#define BUFFER_H_

typedef struct element element_t;

struct element {
	int * data;
	element_t * prev;
	element_t * next;
};
/* elemented_list_t contains a elemented list. */

typedef struct element_list {
	int count;
	element_t * top;
	element_t * bottom;
} element_list_t;

/**
 * Initialises the buffer.
 */
void buffer_init();

/**
 * pushes the given element to the top of the buffer.
 * @param data reference to the value
 */
void buffer_push(int * data);

/**
 * Pops the bottom element (oldest element, FIFO) from the buffer.
 * @return the bottom element (oldest element) of the buffer
 */
int buffer_pop();

/**
 * Returns the current element count of the buffer.
 * @return number of elements in the buffer.
 */
int buffer_size();

/**
 * Resets the buffer and frees the space.
 * ALL ELEMENTS WILL BE LOST
 */
void buffer_reset();

#endif /* BUFFER_H_ */
