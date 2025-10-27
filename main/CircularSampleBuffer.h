#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#include <Arduino.h>
#include "Utils.h"

template <typename T>
class CircularBuffer {

    // This buffer only allows reading of the latest X values, and discards the tail pointer
public:
    CircularBuffer(size_t size) : size(size), head(0), count(0) {
        buffer = new T[size];
    }

    ~CircularBuffer() {
        delete[] buffer;
    }

    void push(const T& sample) {
        if (isFull()) {
            count--;
        }

        buffer[head] = sample;
        head = (head + 1) % size;
        count++;
    }

    size_t popBatch(T* out, size_t batchSize) {
        size_t popped = 0;
        while (popped < batchSize && count > 0) {
            head = (head - 1) < 0 ? head + size - 1 : head - 1; // Use modulo but there are some issues with typecasting from size_t
            out[popped] = buffer[head];
            count--;
            popped++;
        }

        return popped;
    }

    size_t available() const { return count; }
    bool isFull() const { return count == size; }
    bool isEmpty() const { return count == 0; }

private:
    T* buffer;
    size_t size;
    volatile int head;
    volatile size_t count;
};


#endif
