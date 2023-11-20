/*
 * circular_buffer.h
 *
 *  Created on: Oct. 14, 2023
 *      Author: yifeiliu
 */

#ifndef CIRCULAR_BUFFER_H_
#define CIRCULAR_BUFFER_H_

#include <stdatomic.h>
#include <stdbool.h>

typedef struct CircularBuffer
{
    uint8_t *const data;
    atomic_size_t head;
    atomic_size_t tail;
    const size_t size;
} CircularBuffer;

/* Enqueue an element into the buffer, return false if buffer is full */
bool buffer_enqueue(CircularBuffer *buffer, uint8_t byte);

/* Dequeue an element from the buffer, return false if buffer is empty */
bool buffer_dequeue(CircularBuffer *buffer, uint8_t *out);

/* Return the number of elements in the buffer */
size_t buffer_count(CircularBuffer *buffer);

/* Peek [size] elements to out[] without consuming. Return false if [size] > count */
bool buffer_peek(CircularBuffer *buffer, uint8_t *out, size_t size);

/* Skip [num] elements from the buffer, return false if not enough elements */
bool buffer_skip(CircularBuffer *buffer, size_t num);

#endif /* CIRCULAR_BUFFER_H_ */
