#include "deque.h"
	
int main(void){
	DEQUETYPE q;

	init_deque(&q);
	for (int i = 0; i < 3; i++) {
		add_front(&q, i);
		deque_print(&q);
	}
	for (int i = 0; i < 3; i++) {
		int item = delete_rear(&q);
		printf("²¨³½ °ª : %d\n", item);
		deque_print(&q);
	}
	return 0;
}