#ifndef __QUEUE_H
#define __QUEUE_H


#define QNODE_DATA_BYTES 3


//
//  Structure used to define fonts
//
struct Queue {
	struct QNode *front, *rear;
};

struct QNode {

	uint8_t *data;
	struct QNode *next;
};



struct Queue* createQueue();
uint8_t* enQueue(struct Queue *q, uint8_t size);
struct QNode* deQueue(struct Queue *q);


#endif  // _FONTS_H
