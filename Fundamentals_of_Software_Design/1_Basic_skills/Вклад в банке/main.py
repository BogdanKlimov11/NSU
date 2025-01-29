def calculate_deposit(amount, months, annual_rate):
    monthly_rate = annual_rate / 12 / 100
    simple_interest = amount
    compound_interest = amount

    print(f"{'Месяц':<6} {'Простые %':<15} {'Сложные %':<15}")
    print("-" * 40)

    for month in range(1, months + 1):
        simple_interest += amount * (annual_rate / 100) / 12
        compound_interest *= (1 + monthly_rate)
        print(f"{month:<6} {simple_interest:<15.2f} {compound_interest:<15.2f}")

amount = float(input("Введите сумму вклада: "))
months = int(input("Введите срок вклада (в месяцах): "))
annual_rate = float(input("Введите годовой процент: "))

calculate_deposit(amount, months, annual_rate)
