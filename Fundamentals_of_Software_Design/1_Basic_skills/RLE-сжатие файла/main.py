import sys

# Префикс для сжатия/распаковки
PREFIX = b'\xFF'

def compress(input_file, output_file):
    """Функция для сжатия файла с использованием алгоритма RLE."""
    with open(input_file, 'rb') as infile, open(output_file, 'wb') as outfile:
        data = infile.read()

        i = 0
        while i < len(data):
            byte = data[i]
            count = 1

            # Подсчитываем количество одинаковых байтов подряд
            while i + 1 < len(data) and data[i + 1] == byte:
                i += 1
                count += 1

            # Если количество повторений больше 1, заменяем на PF CC BB
            if count > 1:
                outfile.write(PREFIX)  # Пишем префикс
                outfile.write(bytes([count]))  # Пишем количество
                outfile.write(bytes([byte]))  # Пишем сам байт
            else:
                # Если повторений только 1, пишем сам байт
                outfile.write(bytes([byte]))

            i += 1

def decompress(input_file, output_file):
    """Функция для распаковки файла с использованием алгоритма RLE."""
    with open(input_file, 'rb') as infile, open(output_file, 'wb') as outfile:
        data = infile.read()

        i = 0
        while i < len(data):
            if data[i:i+1] == PREFIX:
                # Если встречен префикс, извлекаем количество и байт
                count = data[i + 1]
                byte = data[i + 2]
                outfile.write(byte * count)  # Пишем повторяющиеся байты
                i += 3  # Переходим к следующей части данных
            else:
                # Если префикса нет, просто копируем байт
                outfile.write(data[i:i+1])
                i += 1

def main():
    if len(sys.argv) != 4:
        print("Использование: python3 rle.py c infile outfile — сжатие")
        print("Использование: python3 rle.py d infile outfile — распаковка")
        sys.exit(1)

    mode = sys.argv[1]
    input_file = sys.argv[2]
    output_file = sys.argv[3]

    if mode == 'c':
        compress(input_file, output_file)
        print(f"Файл {input_file} успешно сжат в {output_file}")
    elif mode == 'd':
        decompress(input_file, output_file)
        print(f"Файл {input_file} успешно распакован в {output_file}")
    else:
        print("Неверный режим. Используйте 'c' для сжатия или 'd' для распаковки.")
        sys.exit(1)

if __name__ == '__main__':
    main()
