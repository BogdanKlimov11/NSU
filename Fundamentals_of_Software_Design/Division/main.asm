section .data
    prompt1 db "Enter A: ", 0
    prompt2 db "Enter B: ", 0
    quotient_msg db "Quotient: ", 0
    remainder_msg db "Remainder: ", 0

section .bss
    A resb 4
    B resb 4
    quotient resb 4
    remainder resb 4

section .text
    global _start

_start:
    ; Ввод A
    mov eax, 4
    mov ebx, 1
    mov ecx, prompt1
    mov edx, 9
    int 0x80

    mov eax, 3
    mov ebx, 0
    mov ecx, A
    mov edx, 4
    int 0x80

    ; Ввод B
    mov eax, 4
    mov ebx, 1
    mov ecx, prompt2
    mov edx, 9
    int 0x80

    mov eax, 3
    mov ebx, 0
    mov ecx, B
    mov edx, 4
    int 0x80

    ; Преобразование ASCII в числа
    mov eax, [A]
    sub eax, '0'   ; A = число
    mov ebx, [B]
    sub ebx, '0'   ; B = число

    ; Проверка деления на ноль
    test ebx, ebx
    jz .zero_division

    ; Инициализация
    xor ecx, ecx   ; quotient = 0
    mov edx, eax   ; remainder = A

.loop:
    cmp edx, ebx   ; Если A < B, выходим
    jl .done
    sub edx, ebx   ; A -= B
    inc ecx        ; quotient++
    jmp .loop

.done:
    ; Сохранение результата
    mov [quotient], ecx
    mov [remainder], edx

    ; Вывод частного
    mov eax, 4
    mov ebx, 1
    mov ecx, quotient_msg
    mov edx, 10
    int 0x80

    mov eax, 4
    mov ebx, 1
    mov ecx, quotient
    mov edx, 4
    int 0x80

    ; Вывод остатка
    mov eax, 4
    mov ebx, 1
    mov ecx, remainder_msg
    mov edx, 11
    int 0x80

    mov eax, 4
    mov ebx, 1
    mov ecx, remainder
    mov edx, 4
    int 0x80

    ; Завершение программы
    mov eax, 1
    xor ebx, ebx
    int 0x80

.zero_division:
    mov eax, 4
    mov ebx, 1
    mov ecx, "Error: division by zero!", 0
    mov edx, 24
    int 0x80

    ; Завершение
    mov eax, 1
    xor ebx, ebx
    int 0x80
