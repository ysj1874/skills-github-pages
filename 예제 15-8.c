#include <stdio.h>
#include <string.h>

// 함수 구현
int add(int x, int y) { return x + y; }
int sub(int x, int y) { return x - y; }
int mul(int x, int y) { return x * y; }
int div(int x, int y) { return y != 0 ? x / y : 0; }

// 함수 포인터 타입 정의 type define
typedef int (*operation_t)(int, int);

// 구조체 ?
typedef struct {
	char name[10];
	operation_t func;
} Command;

int main() {
	// 명령어와 함수 포인터 매핑
	Command commands[] = {
		{"add", add},
		{"sub", sub},
		{"mul", mul},
		{"div", div},
		{"", NULL}
	};
	char command[20]; // 명령어 입력
	int a, b;
	printf("원하는 명령을 입력하세요 (예: add 3 5) :");
	scanf_s("%s%d%d", command, &a, &b);
	// 명령어 검색 후 실행
	for (int i = 0; commands[i].func != NULL; i++) {
		if (strcmp(command, commands[i].name) == 0) {
			int result = commands[i].func(a, b);
			printf("결과: %d\n", result);
			return 0;
		}
	}
	// 명령어를 잘못 입력한 경우
	printf("잘못된 명령입니다.\n");
	return 1;
}