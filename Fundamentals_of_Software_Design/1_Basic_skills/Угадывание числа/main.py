def guess_number():
    print("Загадайте число от 1 до 100.")
    low = 1
    high = 100

    while True:
        mid = (low + high) // 2
        answer = input(f"Загаданное число больше или равно {mid}? (д/н): ").strip().lower()

        if answer == "д":
            low = mid
        elif answer == "н":
            high = mid - 1
        else:
            print("Некорректный ввод, пожалуйста, введите 'д' или 'н'.")
            continue

        if low == high:
            print(f"Загаданное число - {low}")
            break

def main():
    while True:
        guess_number()
        play_again = input("Ещё один раунд? (д/н): ").strip().lower()
        if play_again != "д":
            print("Спасибо за игру!")
            break

if __name__ == "__main__":
    main()
