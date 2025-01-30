import sys

def count_file_stats(filename):
    try:
        with open(filename, 'r', encoding='utf-8') as file:
            char_count = 0
            line_count = 0
            word_count = 0

            for line in file:
                line_count += 1
                char_count += len(line)
                word_count += len([word for word in line.split() if word])

        return char_count, line_count, word_count
    except FileNotFoundError:
        print(f"Ошибка: файл '{filename}' не найден.")
        return None

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Использование: python script.py <имя_файла>")
    else:
        filename = sys.argv[1]
        result = count_file_stats(filename)
        if result:
            chars, lines, words = result
            print(f"Символов: {chars}")
            print(f"Строк: {lines}")
            print(f"Слов: {words}")
