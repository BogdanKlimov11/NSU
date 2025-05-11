section .data
    prompt1 db 'Enter the first number: ', 0
    prompt2 db 'Enter the second number: ', 0
    prompt3 db 'Choose operation (1=Sum, 2=Subtraction, 3=Multiplication, 4=Division, 5=Modulus): ', 0
    msg db 'Result: ', 0

section .bss
    num1 resb 4  ; переменная для первого числа
    num2 resb 4  ; переменная для второго числа
    result resb 4  ; переменная для результата

section .text
    global _start

_start:
    ; Ввод первого числа
    mov eax, 4                 ; syscall номер 4 (sys_write)
    mov ebx, 1                 ; дескриптор файла 1 (stdout)
    mov ecx, prompt1           ; адрес строки
    mov edx, 23                ; длина строки
    int 0x80                   ; системный вызов

    mov eax, 3                 ; syscall номер 3 (sys_read)
    mov ebx, 0                 ; дескриптор файла 0 (stdin)
    mov ecx, num1              ; адрес для хранения числа
    mov edx, 4                 ; количество байт для чтения
    int 0x80                   ; системный вызов

    ; Ввод второго числа
    mov eax, 4                 ; syscall номер 4 (sys_write)
    mov ebx, 1                 ; дескриптор файла 1 (stdout)
    mov ecx, prompt2           ; адрес строки
    mov edx, 25                ; длина строки
    int 0x80                   ; системный вызов

    mov eax, 3                 ; syscall номер 3 (sys_read)
    mov ebx, 0                 ; дескриптор файла 0 (stdin)
    mov ecx, num2              ; адрес для хранения числа
    mov edx, 4                 ; количество байт для чтения
    int 0x80                   ; системный вызов

    ; Ввод выбора операции
    mov eax, 4                 ; syscall номер 4 (sys_write)
    mov ebx, 1                 ; дескриптор файла 1 (stdout)
    mov ecx, prompt3           ; адрес строки
    mov edx, 72                ; длина строки
    int 0x80                   ; системный вызов

    mov eax, 3                 ; syscall номер 3 (sys_read)
    mov ebx, 0                 ; дескриптор файла 0 (stdin)
    mov ecx, result            ; адрес для хранения выбора
    mov edx, 4                 ; количество байт для чтения
    int 0x80                   ; системный вызов

    ; Преобразуем введенное значение в число (в нашем случае из строки в число)

    ; Проводим выбор операции, в зависимости от введенной команды
    mov al, [result]           ; загрузим выбранную операцию
    sub al, '0'                ; преобразуем ASCII в число

    cmp al, 1                  ; если 1, то сложение
    je .sum

    cmp al, 2                  ; если 2, то вычитание
    je .sub

    cmp al, 3                  ; если 3, то умножение
    je .mul

    cmp al, 4                  ; если 4, то деление
    je .div

    cmp al, 5                  ; если 5, то остаток от деления
    je .mod

    jmp .done                  ; если ввели неправильный выбор, выходим

.sum:
    ; Сложение
    mov eax, [num1]            ; загружаем первое число
    add eax, [num2]            ; прибавляем второе число
    mov [result], eax          ; сохраняем результат
    jmp .print_result

.sub:
    ; Вычитание
    mov eax, [num1]            ; загружаем первое число
    sub eax, [num2]            ; вычитаем второе число
    mov [result], eax          ; сохраняем результат
    jmp .print_result

.mul:
    ; Умножение
    mov eax, [num1]            ; загружаем первое число
    imul eax, [num2]           ; умножаем на второе число
    mov [result], eax          ; сохраняем результат
    jmp .print_result

.div:
    ; Деление
    mov eax, [num1]            ; загружаем первое число
    xor edx, edx               ; очищаем регистр edx (для деления)
    div dword [num2]           ; делим на второе число (eax:edx) -> eax = результат, edx = остаток
    mov [result], eax          ; сохраняем результат
    jmp .print_result

.mod:
    ; Остаток от деления
    mov eax, [num1]            ; загружаем первое число
    xor edx, edx               ; очищаем edx
    div dword [num2]           ; делим на второе число
    mov [result], edx          ; сохраняем остаток в result
    jmp .print_result

.print_result:
    ; Выводим результат
    mov eax, 4                 ; syscall номер 4 (sys_write)
    mov ebx, 1                 ; дескриптор файла 1 (stdout)
    mov ecx, msg               ; адрес строки
    mov edx, 8                 ; длина строки
    int 0x80                   ; системный вызов

    mov eax, 4                 ; syscall номер 4 (sys_write)
    mov ebx, 1                 ; дескриптор файла 1 (stdout)
    mov ecx, result            ; адрес результата
    mov edx, 4                 ; количество байт для вывода
    int 0x80                   ; системный вызов

.done:
    ; Завершаем программу
    mov eax, 1                 ; syscall номер 1 (sys_exit)
    xor ebx, ebx               ; код возврата 0
    int 0x80                   ; системный вызов
