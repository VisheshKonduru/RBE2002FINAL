#ifndef QUEUE_H
#define QUEUE_H

template <typename T>
class Queue {
private:
    static const int MAX_SIZE = 100; // Adjust size as needed
    T data[MAX_SIZE];
    int front, rear, size;

public:
    Queue() : front(0), rear(-1), size(0) {}

    void enqueue(T element) {
        if (size == MAX_SIZE) {
            // Handle overflow
            return;
        }
        rear = (rear + 1) % MAX_SIZE;
        data[rear] = element;
        size++;
    }

    T dequeue() {
        if (isEmpty()) {
            // Handle underflow
            return T(); // Return default value of T
        }
        T temp = data[front];
        front = (front + 1) % MAX_SIZE;
        size--;
        return temp;
    }

    T peek() const {
        if (isEmpty()) {
            // Handle empty queue
            return T(); // Return default value of T
        }
        return data[front];
    }

    bool isEmpty() const {
        return size == 0;
    }

    int getSize() const {
        return size;
    }
};

#endif // QUEUE_H
