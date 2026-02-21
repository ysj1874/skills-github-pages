#include <stdio.h>

void input_data(int* pa, int* pb);
void swap_data(void);
void print_data(int a, int b);

int a, b;

int main(void)
{
	input_data(&a, &b);
	swap_data();
	print_data(a, b);

	return 0;
}

void input_data(int* pa, int* pb) {
	printf("µÎ Á¤¼ö ÀÔ·Â : ");
	scanf_s("%d%d", pa, pb);
}

void swap_data() {
	int temp;
	temp = a;
	a = b;
	b = temp;
}

void print_data(int a, int b) {
	printf("µÎ Á¤¼ö Ãâ·Â : %d, %d", a, b);
}
