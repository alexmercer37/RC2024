#ifndef _QUEUE_H
#define _QUEUE_H

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

typedef struct QUEUE
{
	float* pdata;
	int front;
	int rear;
	int maxsize;
}QUEUE;

float* CreateQueue(QUEUE* Q, int size);
void Enqueue(QUEUE* Q, float data);
void Dequeue(QUEUE* Q);
float GetData(QUEUE* Q, int number);

#endif