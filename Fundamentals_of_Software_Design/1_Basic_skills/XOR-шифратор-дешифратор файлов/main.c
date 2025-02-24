#include <stdio.h>
#include <string.h>

void xor_encrypt_decrypt(FILE *infile, FILE *outfile, const unsigned char *key, size_t key_len) {
    size_t i = 0;
    unsigned char byte;
    
    while (fread(&byte, 1, 1, infile) == 1) {
        // Шифруем/дешифруем байт с помощью XOR
        byte ^= key[i % key_len];
        fwrite(&byte, 1, 1, outfile);
        i++;
    }
}

int main() {
    char input_file[256], output_file[256], key[256];
    
    // Получаем имена файлов и ключ от пользователя
    printf("Введите имя входного файла: ");
    scanf("%s", input_file);
    printf("Введите имя выходного файла: ");
    scanf("%s", output_file);
    printf("Введите ключ: ");
    scanf("%s", key);
    
    // Открываем файлы
    FILE *infile = fopen(input_file, "rb");
    if (!infile) {
        perror("Ошибка открытия входного файла");
        return 1;
    }
    
    FILE *outfile = fopen(output_file, "wb");
    if (!outfile) {
        perror("Ошибка открытия выходного файла");
        fclose(infile);
        return 1;
    }
    
    // Получаем длину ключа
    size_t key_len = strlen(key);
    
    // Вызываем функцию шифрования/дешифрования
    xor_encrypt_decrypt(infile, outfile, (unsigned char *)key, key_len);
    
    // Закрываем файлы
    fclose(infile);
    fclose(outfile);
    
    printf("Файл %s был успешно обработан в %s.\n", input_file, output_file);
    return 0;
}
