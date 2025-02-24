#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Строка с Base64-символами
const char base64_chars[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

// Функция для кодирования
void encode_base64(FILE *input, FILE *output) {
    unsigned char in[3];
    unsigned char out[4];
    int i = 0;
    size_t num_bytes;

    while ((num_bytes = fread(in, 1, 3, input)) > 0) {
        out[0] = (in[0] >> 2) & 0x3F;
        out[1] = ((in[0] & 0x03) << 4) | ((in[1] >> 4) & 0x0F);
        out[2] = ((in[1] & 0x0F) << 2) | ((in[2] >> 6) & 0x03);
        out[3] = in[2] & 0x3F;

        if (num_bytes < 3) out[3] = 64;  // 64 означает '=' для заполнения

        if (num_bytes < 2) out[2] = 64;

        for (i = 0; i < 4; i++) {
            fputc(base64_chars[out[i]], output);
        }
    }
}

// Функция для декодирования
void decode_base64(FILE *input, FILE *output) {
    unsigned char in[4], out[3];
    int i = 0, j = 0;
    size_t num_bytes;

    while ((num_bytes = fread(in, 1, 4, input)) > 0) {
        for (i = 0; i < 4; i++) {
            if (in[i] == '=') in[i] = 0;
            else in[i] = strchr(base64_chars, in[i]) - base64_chars;
        }

        out[0] = (in[0] << 2) | (in[1] >> 4);
        out[1] = (in[1] << 4) | (in[2] >> 2);
        out[2] = (in[2] << 6) | in[3];

        for (i = 0; i < num_bytes - 1; i++) {
            fputc(out[i], output);
        }
    }
}

int main(int argc, char *argv[]) {
    if (argc != 4) {
        printf("Использование: base64 -e infile outfile для кодирования или base64 -d infile outfile для декодирования\n");
        return 1;
    }

    FILE *input = fopen(argv[2], "rb");
    FILE *output = fopen(argv[3], "wb");

    if (!input || !output) {
        perror("Ошибка открытия файла");
        return 1;
    }

    if (strcmp(argv[1], "-e") == 0) {
        encode_base64(input, output);
        printf("Файл успешно закодирован.\n");
    } else if (strcmp(argv[1], "-d") == 0) {
        decode_base64(input, output);
        printf("Файл успешно декодирован.\n");
    } else {
        printf("Неверный режим. Используйте -e для кодирования или -d для декодирования.\n");
    }

    fclose(input);
    fclose(output);

    return 0;
}
