#include "linked_list.h"

// 오류 처리 함수
void error(char* message) {
	fprintf(stderr, "%s\n", message);
	exit(1);
}

// 리스트 초기화 함수
void init(ArrayListType* L) {
	L->size = 0;
}

// 공백 검사
int is_empty(ArrayListType* L) {
	return L->size == 0;
}

// 포화 검사
int is_full(ArrayListType* L) {
	return L->size == MAX_LIST_SIZE;
}

// 특정 위치의 요소 반환
element get_entry(ArrayListType* L, int pos) {
	if (pos < 0 || pos >= L->size) error("위치 오류");
	return L->array[pos];
}

void insert_last(ArrayListType* L, element item) {
	if (L->size >= MAX_LIST_SIZE) error("리스트 오버플로우");
	L->array[L->size++] = item;
}

void insert(ArrayListType* L, int pos, element item) {
	if (!is_full(L) && (pos >= 0) && (pos <= L->size)) {
		for (int i = (L->size - 1); i >= pos; i--) {
			L->array[i + 1] = L->array[i];
		}
		L->array[pos] = item;
		L->size++;
	}
}

element delete(ArrayListType* L, int pos) {
	element item;
	if (pos < 0 || pos >= L->size) error("위치 오류");
	item = L->array[pos];
	for (int i = pos; i < (L->size - 1); i++) {
		L->array[i] = L->array[i + 1];
	}
	L->size--;
	return item;
}

void print_list(ArrayListType* L) {
	int i;
	for (i = 0; i < L->size; i++)
		printf("%d->", L->array[i]);
	printf("\n");
}