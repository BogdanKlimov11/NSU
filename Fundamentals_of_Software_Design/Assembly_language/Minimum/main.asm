section .data
    prompt db 'Enter three numbers: ', 0   ; Строка подсказки
    msg db 'The minimum is: ', 0           ; Сообщение для вывода минимального числа

section .bss
    num1 resb 4   ; Переменная для первого числа
    num2 resb 4   ; Переменная для второго числа
    num3 resb 4   ; Переменная для третьего числа
    min resb 4    ; Переменная для минимального числа

section .text
    global _start  ; Точка входа программы

_start:
    ; Выводим подсказку
    mov eax, 4         ; syscall номер 4 (sys_write)
    mov ebx, 1         ; дескриптор файла 1 (stdout)
    mov ecx, prompt    ; адрес строки
    mov edx, 20        ; длина строки
    int 0x80           ; системный вызов

    ; Вводим три числа
    ; Чтение первого числа
    mov eax, 3         ; syscall номер 3 (sys_read)
    mov ebx, 0         ; дескриптор файла 0 (stdin)
    mov ecx, num1      ; адрес для хранения числа
    mov edx, 4         ; количество байт для чтения
    int 0x80           ; системный вызов

    ; Чтение второго числа
    mov eax, 3         ; syscall номер 3 (sys_read)
    mov ebx, 0         ; дескриптор файла 0 (stdin)
    mov ecx, num2      ; адрес для хранения числа
    mov edx, 4         ; количество байт для чтения
    int 0x80           ; системный вызов

    ; Чтение третьего числа
    mov eax, 3         ; syscall номер 3 (sys_read)
    mov ebx, 0         ; дескриптор файла 0 (stdin)
    mov ecx, num3      ; адрес для хранения числа
    mov edx, 4         ; количество байт для чтения
    int 0x80           ; системный вызов

    ; Нахождение минимального числа
    ; Сравниваем num1 и num2
    mov eax, [num1]
    mov ebx, [num2]
    cmp eax, ebx
    jle .min1isfirst   ; если num1 <= num2, переходим к следующему сравнению
    mov eax, ebx       ; иначе num2 становится минимальным
.min1isfirst:
    
    ; Сравниваем num1 или num2 с num3
    mov ebx, [num3]
    cmp eax, ebx
    jle .done          ; если текущее минимальное значение <= num3, завершаем
    mov eax, ebx       ; иначе num3 становится минимальным

.done:
    ; Выводим минимальное значение
    mov [min], eax

    ; Выводим сообщение о минимальном числе
    mov eax, 4         ; syscall номер 4 (sys_write)
    mov ebx, 1         ; дескриптор файла 1 (stdout)
    mov ecx, msg       ; адрес строки
    mov edx, 18        ; длина строки
    int 0x80           ; системный вызов

    ; Выводим минимальное число
    mov eax, 4         ; syscall номер 4 (sys_write)
    mov ebx, 1         ; дескриптор файла 1 (stdout)
    mov ecx, min       ; адрес минимального числа
    mov edx, 4         ; количество байт для вывода
    int 0x80           ; системный вызов

    ; Завершаем программу
    mov eax, 1         ; syscall номер 1 (sys_exit)
    xor ebx, ebx       ; код возврата 0
    int 0x80           ; системный вызов
