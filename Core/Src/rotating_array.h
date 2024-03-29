#ifndef __ROTATING_ARRAY_H
#define __ROTATING_ARRAY_H

#include "stm32f0xx_hal.h"

typedef uint16_t rot_arr_size_t;
typedef uint32_t rot_arr_cont_t;

// Defines a rotating array with the given size.
// Create an instance of this struct and set the 
// size and content properties, then call
// rot_arr_init
struct RotArr_t {
	rot_arr_size_t size;
	rot_arr_cont_t* contents;
	rot_arr_size_t _start;
	rot_arr_size_t _end;
};

// Initializes the internal properties of the 
// rotating array.
void rot_arr_init (struct RotArr_t* arr) {
	arr->_start = 0;
	arr->_end = 0;
	
}

// Adds a new item to the array, wrapping around 
// to the start if needed.
void rot_arr_add (struct RotArr_t* arr, rot_arr_cont_t item) {
	arr->_end = (arr->_end + 1) % arr->size;
	arr->contents[arr->_end] = item;
	if (arr->_end == arr->_start) {
		arr->_start = (arr->_start + 1) % arr->size;
	}
}

rot_arr_cont_t rot_arr_get (struct RotArr_t* arr, rot_arr_size_t index, char* status) {
	*status = 1;
	if (index < arr->size) {
		*status = 0;
	}
	rot_arr_size_t i = (arr->_start + index) % arr->size;
	return arr->contents[i];
}

rot_arr_cont_t rot_arr_top (struct RotArr_t* arr) {
	return arr->contents[arr->_end];
}

#endif