#include "../inc/queue.h"

// 实现循环队列的创建、是否为满/空、入队和出队

float *CreateQueue(QUEUE *Q, int size)
{
	Q->pdata = (float *)malloc(sizeof(float) * size);
	if (Q->pdata == NULL)
		exit(-1);
	Q->front = 0;
	Q->rear = 0;
	Q->maxsize = size;
	return Q->pdata;
}

bool IfFull(QUEUE *Q)
{
	if ((Q->rear + 1) % (Q->maxsize) == Q->front)
		return 1;
	else
		return 0;
}

bool IfEmpty(QUEUE *Q)
{
	if (Q->front == Q->rear)
		return 1;
	else
		return 0;
}

void Enqueue(QUEUE *Q, float data)
{
	if (IfFull(Q))
		printf("入队失败！\n");
	else
	{
		Q->pdata[Q->rear] = data;
		Q->rear = (Q->rear + 1) % Q->maxsize;
	}
}

void Dequeue(QUEUE *Q)
{
	if (IfEmpty(Q))
		printf("队列空，出队失败！\n");
	else
		Q->front = (Q->front + 1) % Q->maxsize;
}

float GetData(QUEUE *Q, int number)
{
	int i = Q->front;
	return Q->pdata[(i + number) % Q->maxsize];
}
