def xor_encrypt_decrypt(input_file, output_file, key):
    # Открываем входной и выходной файлы
    with open(input_file, 'rb') as infile, open(output_file, 'wb') as outfile:
        i = 0  # Индекс для ключа
        key_len = len(key)
        
        # Читаем байты входного файла по одному и записываем в выходной
        while (byte := infile.read(1)):
            # Шифруем/дешифруем каждый байт с использованием XOR
            encrypted_byte = byte[0] ^ key[i % key_len]
            outfile.write(bytes([encrypted_byte]))
            i += 1


if __name__ == '__main__':
    # Получаем путь к файлам и ключ от пользователя
    input_file = input("Введите имя входного файла: ")
    output_file = input("Введите имя выходного файла: ")
    key = input("Введите ключ: ").encode('utf-8')

    # Вызываем функцию для шифрования/дешифрования
    xor_encrypt_decrypt(input_file, output_file, key)
    print(f"Файл {input_file} был успешно обработан в {output_file}.")
