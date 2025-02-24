#include <stdio.h>
#include <assert.h>
#include <stddef.h>

void bitcpy(unsigned char *dst, size_t dst_ofs, unsigned char *src, size_t src_ofs, size_t bit_count) {
    size_t dst_byte_ofs = dst_ofs / 8;  // байт, в котором начинается целевая цепочка
    size_t dst_bit_ofs = dst_ofs % 8;   // сдвиг внутри байта
    size_t src_byte_ofs = src_ofs / 8;  // байт, в котором начинается исходная цепочка
    size_t src_bit_ofs = src_ofs % 8;   // сдвиг внутри байта

    size_t bits_copied = 0;  // счетчик скопированных битов

    while (bits_copied < bit_count) {
        unsigned char src_bit = (src[src_byte_ofs] >> (7 - src_bit_ofs)) & 1;  // получаем бит из исходного буфера
        if (dst_bit_ofs == 0) {
            dst[dst_byte_ofs] &= ~(1 << 7);  // очищаем старший бит
            dst[dst_byte_ofs] |= (src_bit << 7);  // копируем бит в старший бит
        } else {
            dst[dst_byte_ofs] &= ~(1 << (7 - dst_bit_ofs));  // очищаем соответствующий бит
            dst[dst_byte_ofs] |= (src_bit << (7 - dst_bit_ofs));  // копируем бит
        }

        // Увеличиваем счетчик битов
        bits_copied++;

        // Переходим к следующему биту
        src_bit_ofs++;
        if (src_bit_ofs == 8) {
            src_bit_ofs = 0;
            src_byte_ofs++;
        }

        dst_bit_ofs++;
        if (dst_bit_ofs == 8) {
            dst_bit_ofs = 0;
            dst_byte_ofs++;
        }
    }
}

int main() {
    // Тест 1: Копируем 7 бит из второго байта первого в второй
    unsigned char src[2] = { 0x66, 0xFD };  // Исходная цепочка: 01100110 11111101
    unsigned char dst[2] = { 0xE0, 0x00 };  // Целевая цепочка: 00000111 00000000

    // Копируем 7 бит начиная с 8-го бита во втором байте исходного
    bitcpy(dst, 2, src, 8, 7);  // Ожидаем: 11110111 00000001

    assert(dst[0] == 0xF7 && dst[1] == 0x01);  // Проверка

    // Тест 2: Копируем 8 бит из первого байта исходного
    unsigned char src2[2] = { 0xAA, 0x00 };  // Исходная цепочка: 10101010 00000000
    unsigned char dst2[2] = { 0x00, 0x00 };  // Целевая цепочка: 00000000 00000000

    // Копируем 8 бит начиная с 0-го бита первого байта
    bitcpy(dst2, 0, src2, 0, 8);  // Ожидаем: 10101010 00000000

    assert(dst2[0] == 0xAA && dst2[1] == 0x00);  // Проверка

    printf("Все тесты пройдены успешно!\n");
    return 0;
}
