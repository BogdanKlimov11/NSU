#include <stdio.h>

long long fibonacci(int n) {
    if (n <= 0) return 0;
    if (n == 1) return 1;

    long long a = 0, b = 1, temp;
    for (int i = 2; i <= n; i++) {
        temp = a + b;
        a = b;
        b = temp;
    }
    return b;
}

int main() {
    int n;
    printf("Введите n: ");
    scanf("%d", &n);
    printf("F(%d) = %lld\n", n, fibonacci(n));
    return 0;
}
