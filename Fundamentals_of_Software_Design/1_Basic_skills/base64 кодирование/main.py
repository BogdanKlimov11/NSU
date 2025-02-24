import base64
import sys

def encode_base64(input_file, output_file):
    """Функция для кодирования файла в Base64 с разбивкой на строки длиной не более 72 символов."""
    with open(input_file, "rb") as infile:
        data = infile.read()

    encoded_data = base64.b64encode(data).decode("utf-8")
    
    # Разбиваем на строки длиной не более 72 символов
    encoded_data_lines = [encoded_data[i:i+72] for i in range(0, len(encoded_data), 72)]

    with open(output_file, "w") as outfile:
        for line in encoded_data_lines:
            outfile.write(line + "\n")

def decode_base64(input_file, output_file):
    """Функция для декодирования Base64 данных из файла."""
    with open(input_file, "r") as infile:
        encoded_data = infile.read().replace("\n", "")  # Убираем переводы строк

    decoded_data = base64.b64decode(encoded_data)

    with open(output_file, "wb") as outfile:
        outfile.write(decoded_data)

if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Использование: base64 -e infile outfile для кодирования или base64 -d infile outfile для декодирования")
        sys.exit(1)

    mode = sys.argv[1]
    input_file = sys.argv[2]
    output_file = sys.argv[3]

    if mode == "-e":
        encode_base64(input_file, output_file)
        print(f"Файл {input_file} успешно закодирован в {output_file}")
    elif mode == "-d":
        decode_base64(input_file, output_file)
        print(f"Файл {input_file} успешно декодирован в {output_file}")
    else:
        print("Неверный режим. Используйте -e для кодирования или -d для декодирования.")
        sys.exit(1)
