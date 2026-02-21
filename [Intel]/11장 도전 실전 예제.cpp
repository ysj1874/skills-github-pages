#include <stdio.h>

int main() {
    int ch;
    int max_word_length = 0;
    int cnt = 0;
    int loop = 1;

    do {
        do {
            ch = getchar();
            if (ch != '\n') cnt++;
            if (ch == -1) loop = 0;
        } while (ch != '\n');
        if (cnt > max_word_length) max_word_length = cnt;
        cnt = 0;
    } while (loop);
    printf("°¡Àå ±ä ´Ü¾îÀÇ ±æÀÌ : %d", max_word_length);

    return 0;
}
