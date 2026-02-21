#include <stdio.h>
int main(void)
{
	int ch;
	int cnt = 0;
	ch - getchar();

	while (ch != '\n') {
		if (ch >= 'a') cnt++;
		ch = getchar();
	}
	printf("¼Ò¹®ÀÚÀÇ °³¼ö : %d\n", cnt);

	return 0;
}
