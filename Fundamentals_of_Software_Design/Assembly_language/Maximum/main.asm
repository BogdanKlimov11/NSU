section .data
    prompt1 db 'Enter the number of elements (N): ', 0
    prompt2 db 'Enter the numbers: ', 0
    msg db 'Maximum: ', 0

section .bss
    N resb 4                  ; переменная для хранения числа N
    num resb 4                ; переменная для хранения каждого введенного числа
    max resb 4                ; переменная для хранения максимума

section .text
    global _start

_start:
    ; Ввод количества чисел N
    mov eax, 4                 ; syscall номер 4 (sys_write)
    mov ebx, 1                 ; дескриптор файла 1 (stdout)
    mov ecx, prompt1           ; адрес строки
    mov edx, 30                ; длина строки
    int 0x80                   ; системный вызов

    mov eax, 3                 ; syscall номер 3 (sys_read)
    mov ebx, 0                 ; дескриптор файла 0 (stdin)
    mov ecx, N                 ; адрес для хранения N
    mov edx, 4                 ; количество байт для чтения
    int 0x80                   ; системный вызов

    ; Преобразуем N из строки в число
    mov eax, [N]               ; загружаем N
    sub eax, '0'               ; преобразуем из ASCII в число
    mov ebx, eax               ; сохраняем количество чисел в ebx

    ; Ввод чисел и нахождение максимума
    mov eax, 0                 ; начинаем с нулевого максимума
    mov [max], eax             ; сохраняем значение в переменную max

.input_loop:
    ; Выводим запрос на ввод числа
    mov eax, 4                 ; syscall номер 4 (sys_write)
    mov ebx, 1                 ; дескриптор файла 1 (stdout)
    mov ecx, prompt2           ; адрес строки
    mov edx, 15                ; длина строки
    int 0x80                   ; системный вызов

    mov eax, 3                 ; syscall номер 3 (sys_read)
    mov ebx, 0                 ; дескриптор файла 0 (stdin)
    mov ecx, num               ; адрес для хранения введенного числа
    mov edx, 4                 ; количество байт для чтения
    int 0x80                   ; системный вызов

    ; Преобразуем введенное число в число
    mov eax, [num]             ; загружаем число
    sub eax, '0'               ; преобразуем из ASCII в число

    ; Сравниваем с текущим максимумом
    mov ebx, [max]             ; загружаем текущий максимум
    cmp eax, ebx               ; сравниваем введенное число с максимумом
    jle .next                   ; если текущее число меньше или равно, переходим к следующему

    ; Если текущее число больше, обновляем максимум
    mov [max], eax             ; сохраняем новое максимальное значение

.next:
    dec ebx                    ; уменьшаем счетчик чисел
    jnz .input_loop            ; если не все числа введены, продолжаем вводить

    ; Выводим результат
    mov eax, 4                 ; syscall номер 4 (sys_write)
    mov ebx, 1                 ; дескриптор файла 1 (stdout)
    mov ecx, msg               ; адрес строки
    mov edx, 9                 ; длина строки
    int 0x80                   ; системный вызов

    mov eax, 4                 ; syscall номер 4 (sys_write)
    mov ebx, 1                 ; дескриптор файла 1 (stdout)
    mov ecx, max               ; адрес для вывода максимума
    mov edx, 4                 ; количество байт для вывода
    int 0x80                   ; системный вызов

    ; Завершаем программу
    mov eax, 1                 ; syscall номер 1 (sys_exit)
    xor ebx, ebx               ; код возврата 0
    int 0x80                   ; системный вызов
