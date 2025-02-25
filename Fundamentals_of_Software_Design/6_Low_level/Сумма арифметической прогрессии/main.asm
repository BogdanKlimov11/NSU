section .data
    prompt1 db "Enter a1: ", 0
    prompt2 db "Enter d: ", 0
    prompt3 db "Enter N: ", 0
    result_msg db "Sum: ", 0

section .bss
    a1 resb 4
    d resb 4
    N resb 4
    sum resb 4

section .text
    global _start

_start:
    ; Ввод a1
    mov eax, 4
    mov ebx, 1
    mov ecx, prompt1
    mov edx, 9
    int 0x80

    mov eax, 3
    mov ebx, 0
    mov ecx, a1
    mov edx, 4
    int 0x80

    ; Ввод d
    mov eax, 4
    mov ebx, 1
    mov ecx, prompt2
    mov edx, 8
    int 0x80

    mov eax, 3
    mov ebx, 0
    mov ecx, d
    mov edx, 4
    int 0x80

    ; Ввод N
    mov eax, 4
    mov ebx, 1
    mov ecx, prompt3
    mov edx, 9
    int 0x80

    mov eax, 3
    mov ebx, 0
    mov ecx, N
    mov edx, 4
    int 0x80

    ; Инициализация переменных
    mov eax, [a1]  ; a1
    sub eax, '0'   ; Преобразуем ASCII в число
    mov [sum], eax ; sum = a1

    mov ebx, [N]   ; N
    sub ebx, '0'   ; Преобразуем ASCII в число
    dec ebx        ; Уменьшаем, т.к. a1 уже добавили

    mov ecx, [d]   ; d
    sub ecx, '0'   ; Преобразуем ASCII в число

.loop:
    test ebx, ebx  ; Если N == 0, выходим
    jz .done

    add eax, ecx   ; Добавляем d к предыдущему элементу
    add [sum], eax ; Добавляем в сумму
    dec ebx        ; Уменьшаем N
    jmp .loop

.done:
    ; Вывод результата
    mov eax, 4
    mov ebx, 1
    mov ecx, result_msg
    mov edx, 5
    int 0x80

    mov eax, 4
    mov ebx, 1
    mov ecx, sum
    mov edx, 4
    int 0x80

    ; Завершение программы
    mov eax, 1
    xor ebx, ebx
    int 0x80
