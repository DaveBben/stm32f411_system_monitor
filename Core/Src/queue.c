// A C program to demonstrate linked list based implementation of queue
//https://www.geeksforgeeks.org/queue-linked-list-implementation/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// A linked list (LL) node to store a queue entry
struct QNode {

	uint8_t *data;
	struct QNode *next;
};

// The queue, front stores the front node of LL and rear stores the
// last node of LL
struct Queue {
	struct QNode *front, *rear;
};

// A utility function to create an empty queue
struct Queue* createQueue() {
	struct Queue *q = (struct Queue*) malloc(sizeof(struct Queue));
	q->front = q->rear = NULL;
	return q;
}

// The function to add a key k to q
uint8_t* enQueue(struct Queue *q, uint8_t size) {
	if (q) {
		struct QNode *temp = (struct QNode*) malloc(sizeof(struct QNode));
		if(temp == NULL){
			printf("\nHeap Overflow");
			return NULL;
		}
		uint8_t *data = (uint8_t*) malloc(size * sizeof(uint8_t));
		if(data == NULL){
			printf("\nHeap Overflow");
			return NULL;
		}
		temp->data = data;
		temp->next = NULL;

		// If queue is empty, then new node is front and rear both
		if (q->rear == NULL) {
			q->front = q->rear = temp;
		} else {
			// Add the new node at the end of queue and change rear
			q->rear->next = temp;
			q->rear = temp;

		}
		return data;
	}else{
		return NULL;
	}

}

// Function to remove a key from given queue q
struct QNode* deQueue(struct Queue *q) {
	if (q) {
		// If queue is empty, return NULL.
		if (q->front == NULL){
			return NULL;
		}

		// Store previous front and move front one node ahead
		struct QNode *temp = q->front;

		q->front = q->front->next;

		// If front becomes NULL, then change rear also as NULL
		if (q->front == NULL){
			q->rear = NULL;
		}
		return temp;
	}else{
		return NULL;
	}

}

