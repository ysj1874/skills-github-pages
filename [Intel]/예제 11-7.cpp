#include <stdio.h>

int main(void)
{
	int num, grade;

	printf("학번 입력 : ");
	scanf_s("%d", &num);
	getchar();	// getchar() 가 없으면 학점 입력을 받을 수 있지 않음 (개행 제거가 안돼서)
	printf("학점 입력 : ");
	grade = getchar();
	printf("학번 : %d, 학점 : %c", num, grade); \

		return 0;
}
