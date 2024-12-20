# Проект процессора для математических операций на SystemVerilog

## Обзор

Этот проект включает реализацию базового процессора, способного выполнять основные математические операции, такие как сложение, вычитание, умножение и деление. Процессор спроектирован и реализован на **SystemVerilog** с фокусом на следующие компоненты:

- **АЛУ (Арифметико-логическое устройство)**: Выполняет арифметические и логические операции (сложение, вычитание, умножение и деление).
- **Регистр**: Хранит и извлекает данные.
- **Память**: Моделирует операции памяти для чтения и записи данных.
- **Блок управления**: Управляет потоками данных, операциями и управляющими сигналами внутри процессора.

## Особенности

- **Арифметические операции**: АЛУ может выполнять базовые операции, такие как сложение, вычитание, умножение и деление.
- **Регистр**: Набор из 32 регистров для временного хранения данных.
- **Память**: Простая память, поддерживающая операции чтения и записи.
- **Блок управления**: Блок управления генерирует необходимые сигналы для АЛУ, регистров и памяти в зависимости от переданного кода операции (opcode).

## Системные компоненты

### АЛУ (Арифметико-логическое устройство)
АЛУ выполняет следующие операции:
- **Сложение**: Складывает два операнда.
- **Вычитание**: Вычитает второй операнд из первого.
- **Умножение**: Умножает два операнда.
- **Деление**: Делит первый операнд на второй (с обработкой ошибки деления на ноль).

### Регистр
Регистр представляет собой набор из 32 регистров, каждый из которых имеет ширину 32 бита, и служит для хранения операндов и результатов.

### Память
Модуль памяти представляет собой простой массив из 1024 ячеек памяти по 32 бита, с поддержкой операций чтения и записи.

### Блок управления
Блок управления генерирует соответствующие управляющие сигналы в зависимости от переданного кода операции (opcode). Он управляет работой АЛУ, записью в регистры, чтением и записью в память и другими процессами.

## Реализация

Процессор состоит из следующих модулей:

1. **АЛУ**: Выполняет заданную арифметическую или логическую операцию.
2. **Регистры**: Набор регистров для хранения данных в процессе выполнения инструкций.
3. **Память**: Моделирует пространство памяти для чтения и записи данных.
4. **Блок управления**: Декодирует коды операций и генерирует управляющие сигналы для АЛУ и памяти.

### Основной поток работы процессора:
1. Блок управления декодирует код операции и генерирует управляющие сигналы.
2. АЛУ выполняет операцию в соответствии с полученными управляющими сигналами.
3. Результат операции сохраняется в регистре или памяти.
4. Процессор может обрабатывать арифметические операции, операции с памятью и управлять процессами на основе переданных кодов операций.

## Будущие улучшения
- Реализация более сложных математических операций (например, интегрирование, дифференцирование).
- Добавление поддержки операций с плавающей точкой.
- Оптимизация производительности с использованием конвейеризации или других продвинутых техник.
