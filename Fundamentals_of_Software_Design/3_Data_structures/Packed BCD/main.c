#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef unsigned int Bcd;

#define INVALID_BCD 0xFFFFFFFFU
#define BCD_SIGN_PLUS  0xC
#define BCD_SIGN_MINUS 0xD

/* Проверка корректности BCD */
int is_valid_bcd(Bcd n) {
    for (int i = 0; i < 7; i++) {
        if (((n >> (i * 4)) & 0xF) > 9)
            return 0;
    }
    int sign = (n >> 28) & 0xF;
    return sign == BCD_SIGN_PLUS || sign == BCD_SIGN_MINUS;
}

/* Преобразование из int в BCD */
Bcd bcd_from_int(int n) {
    if (n < -9999999 || n > 9999999)
        return INVALID_BCD;
    
    Bcd bcd = 0;
    int sign = (n >= 0) ? BCD_SIGN_PLUS : BCD_SIGN_MINUS;
    if (n < 0) n = -n;
    
    for (int i = 0; i < 7; i++) {
        bcd |= (n % 10) << (i * 4);
        n /= 10;
    }
    
    return bcd | (sign << 28);
}

/* Преобразование из BCD в int */
int bcd_to_int(Bcd n) {
    if (!is_valid_bcd(n)) return 0;
    
    int result = 0;
    int sign = ((n >> 28) & 0xF) == BCD_SIGN_MINUS ? -1 : 1;
    int multiplier = 1;
    
    for (int i = 0; i < 7; i++) {
        result += ((n >> (i * 4)) & 0xF) * multiplier;
        multiplier *= 10;
    }
    
    return result * sign;
}

/* Сложение BCD */
Bcd bcd_add(Bcd n1, Bcd n2) {
    if (!is_valid_bcd(n1) || !is_valid_bcd(n2)) return INVALID_BCD;
    int sum = bcd_to_int(n1) + bcd_to_int(n2);
    return bcd_from_int(sum);
}

/* Вычитание BCD */
Bcd bcd_sub(Bcd n1, Bcd n2) {
    if (!is_valid_bcd(n1) || !is_valid_bcd(n2)) return INVALID_BCD;
    int diff = bcd_to_int(n1) - bcd_to_int(n2);
    return bcd_from_int(diff);
}

/* Умножение BCD */
Bcd bcd_mul(Bcd n1, Bcd n2) {
    if (!is_valid_bcd(n1) || !is_valid_bcd(n2)) return INVALID_BCD;
    int prod = bcd_to_int(n1) * bcd_to_int(n2);
    return bcd_from_int(prod);
}

/* Деление BCD */
void bcd_div(Bcd n1, Bcd n2, Bcd *pq, Bcd *pr) {
    if (!is_valid_bcd(n1) || !is_valid_bcd(n2) || bcd_to_int(n2) == 0) {
        *pq = *pr = 0xC;
        return;
    }
    int q = bcd_to_int(n1) / bcd_to_int(n2);
    int r = bcd_to_int(n1) % bcd_to_int(n2);
    *pq = bcd_from_int(q);
    *pr = bcd_from_int(r);
}

/* Сравнение */
int bcd_compare(Bcd n1, Bcd n2) {
    if (!is_valid_bcd(n1) || !is_valid_bcd(n2)) return 0;
    int i1 = bcd_to_int(n1);
    int i2 = bcd_to_int(n2);
    return (i1 > i2) - (i1 < i2);
}

/* Преобразование BCD в строку */
char *bcd_to_str(Bcd n, char *buf, int bufsize) {
    if (!is_valid_bcd(n) || bufsize < 9) {
        if (bufsize > 0) buf[0] = '\0';
        return NULL;
    }
    int num = bcd_to_int(n);
    snprintf(buf, bufsize, "%d", num);
    return buf;
}

/* Тестирование */
int main() {
    Bcd n1 = bcd_from_int(1234567);
    Bcd n2 = bcd_from_int(-7654321);
    char buf[20];
    
    printf("n1: %s\n", bcd_to_str(n1, buf, sizeof(buf)));
    printf("n2: %s\n", bcd_to_str(n2, buf, sizeof(buf)));
    
    Bcd sum = bcd_add(n1, n2);
    printf("Sum: %s\n", bcd_to_str(sum, buf, sizeof(buf)));
    
    Bcd diff = bcd_sub(n1, n2);
    printf("Diff: %s\n", bcd_to_str(diff, buf, sizeof(buf)));
    
    Bcd prod = bcd_mul(n1, n2);
    printf("Prod: %s\n", bcd_to_str(prod, buf, sizeof(buf)));
    
    Bcd q, r;
    bcd_div(n1, bcd_from_int(123), &q, &r);
    printf("Quotient: %s, Remainder: %s\n", bcd_to_str(q, buf, sizeof(buf)), bcd_to_str(r, buf, sizeof(buf)));
    
    return 0;
}
