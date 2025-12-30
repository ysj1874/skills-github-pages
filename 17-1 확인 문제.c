#include <stdio.h>

struct cracker {
	int price;
	int calories;
};

int main() {
	struct cracker s;
	printf("바사삭의 가격과 열량을 입력하세요 : ");
	scanf_s("%d%d", &(s.price), &(s.calories));
	printf("바사삭의 가격 : %d원\n", s.price);
	printf("바사삭의 열량 : %dkcal", s.calories);
	return 0;
}