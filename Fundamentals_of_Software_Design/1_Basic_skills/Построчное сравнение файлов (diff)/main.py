import sys

def compare_files(file1, file2):
    with open(file1, 'r', encoding='utf-8') as f1, open(file2, 'r', encoding='utf-8') as f2:
        for line_num, (line1, line2) in enumerate(zip(f1, f2), start=1):
            if line1 != line2:
                print(f"Различие в строке {line_num}:")
                print(f"- {file1}: {line1.strip()}")
                print(f"- {file2}: {line2.strip()}")
                return
        if f1.readline() or f2.readline():
            print(f"Файлы имеют разное количество строк.")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Использование: python compare_files.py file1.txt file2.txt")
    else:
        compare_files(sys.argv[1], sys.argv[2])
