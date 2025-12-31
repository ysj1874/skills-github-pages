#include <stdio.h>
#include <stdlib.h>
#define MAX_QUEUE_SIZE	5

typedef int element;
typedef struct {
	int front;
	int rear;
	element data[MAX_QUEUE_SIZE];
}QUEUETYPE;

//오류함수
void error(char* message) {
	fprintf(stderr, "%s\n", message);
	exit(1);
}

void unit_queue(QUEUETYPE* q) {
	q->rear = -1;
	q->front = -1;
}

void queue_print(QUEUETYPE* q) {
	for (int i = 0; i < MAX_QUEUE_SIZE; i++) {
		if (i <= q->front || i > q->rear)
			printf(" | ");
		else
			printf("%d | ", q->data[i]);
	}
	printf("\n");
}

int is_full(QUEUETYPE* q) {
	if (q->rear == MAX_QUEUE_SIZE - 1)
		return 1;
	else
		return 0;
}

int is_empty(QUEUETYPE* q) {
	if (q->front == q->rear)
		return 1;
	else
		return 0;
}

void enqueue(QUEUETYPE*q, int item) {
	if (is_full(q)){
		error("큐가 포화상태입니다.");
	return;
}
q->data[++(q->rear)] = item;
}

int dequeue(QUEUETYPE* q) {
	if (is_empty(q)) {
		error("큐가 공백상태입니다.");
		return -1;
	}
	return q->data[++(q->front)];
}

int main(void) {
	int item = 0;
	QUEUETYPE q;
	
	init_queue(&q);

	enqueue(&q, 10); queue_print(&q);
	enqueue(&q, 20); queue_print(&q);
	enqueue(&q, 30); queue_print(&q);

	item = dequeue(&q); queue_print(&q);
	item = dequeue(&q); queue_print(&q);
	item = dequeue(&q); queue_print(&q);
	return 0;
}