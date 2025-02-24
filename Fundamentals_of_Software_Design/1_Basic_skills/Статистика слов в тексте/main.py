import re
from collections import defaultdict

def process_text(filename):
    with open(filename, 'r', encoding='utf-8') as file:
        text = file.read().lower()
    
    # Удаляем знаки пунктуации в начале и в конце слова
    words = re.findall(r'\b\w+\b', text)
    
    word_count = defaultdict(int)
    
    for word in words:
        word_count[word] += 1
    
    return word_count

def save_statistics(word_count, output_filename):
    with open(output_filename, 'w', encoding='utf-8') as outfile:
        for word, count in word_count.items():
            outfile.write(f"{word} {count}\n")

def main():
    input_filename = "infile.txt"  # Укажите свой файл для подсчета
    output_filename = "outfile.txt"  # Укажите свой файл для записи статистики
    
    word_count = process_text(input_filename)
    save_statistics(word_count, output_filename)
    print(f"Статистика записана в файл: {output_filename}")

if __name__ == "__main__":
    main()
