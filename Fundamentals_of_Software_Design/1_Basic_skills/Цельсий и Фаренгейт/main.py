def celsius_to_fahrenheit(celsius):
    return celsius * 9 / 5 + 32

def fahrenheit_to_celsius(fahrenheit):
    return (fahrenheit - 32) * 5 / 9

def main():
    print("Выберите режим перевода:")
    print("1. Цельсий в Фаренгейт")
    print("2. Фаренгейт в Цельсий")
    
    choice = input("Введите номер режима (1 или 2): ")
    
    if choice == "1":
        celsius = float(input("Введите температуру в градусах Цельсия: "))
        fahrenheit = celsius_to_fahrenheit(celsius)
        print(f"{celsius:.2f}°C = {fahrenheit:.2f}°F")
    elif choice == "2":
        fahrenheit = float(input("Введите температуру в градусах Фаренгейта: "))
        celsius = fahrenheit_to_celsius(fahrenheit)
        print(f"{fahrenheit:.2f}°F = {celsius:.2f}°C")
    else:
        print("Некорректный выбор режима.")

if __name__ == "__main__":
    main()
