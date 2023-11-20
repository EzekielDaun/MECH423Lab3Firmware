/*
 * circular_buffer.c
 *
 *  Created on: Oct. 14, 2023
 *      Author: yifeiliu
 */

#include "circular_buffer.h"

bool buffer_enqueue(CircularBuffer *buffer, uint8_t byte)
{
    size_t tail = atomic_load(&buffer->tail);
    size_t next_tail = (tail + 1) % buffer->size;

    if (next_tail == atomic_load(&buffer->head))
    {
        // The buffer is full
        return false;
    }

    buffer->data[tail] = byte;
    atomic_store(&buffer->tail, next_tail);

    return true;
}

bool buffer_dequeue(CircularBuffer *buffer, uint8_t *out)
{
    size_t head = atomic_load(&buffer->head);
    if (head == atomic_load(&buffer->tail))
    {
        // The buffer is empty
        return false;
    }

    *out = buffer->data[head];
    // buffer->data[head] = '-';
    atomic_store(&buffer->head, (head + 1) % buffer->size);

    return true;
}

size_t buffer_count(CircularBuffer *buffer)
{
    size_t head = atomic_load(&buffer->head);
    size_t tail = atomic_load(&buffer->tail);
    return (buffer->size - head + tail) % (buffer->size);
}

bool buffer_peek(CircularBuffer *buffer, uint8_t *out, size_t size)
{
    size_t buffer_size = buffer->size;
    size_t count = buffer_count(buffer);
    if (count < size)
    {
        return false;
    }

    size_t head = atomic_load(&buffer->head);
    for (size_t i = 0; i < size; ++i)
    {
        out[i] = buffer->data[(head + i) % buffer_size];
    }
    return true;
}

bool buffer_skip(CircularBuffer *buffer, size_t num)
{
    if (buffer_count(buffer) < num)
    {
        return false;
    }

    size_t head = atomic_load(&buffer->head);
    atomic_store(&buffer->head, (head + num) % buffer->size);
    return true;
}
