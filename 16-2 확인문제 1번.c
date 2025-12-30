#include <stdio.h>
#include <stdlib.h>

int main(int argc, char** argv) {
	int** matrix;

	// 행의 저장공간 할당
	matrix = (int**)malloc(4 * sizeof(int*));
	// 열의 저장공간 할당
	for(int i =0;i<4;i++)
	matrix[0] = (int*)malloc(sizeof(int) * 5);
	return 0;
}