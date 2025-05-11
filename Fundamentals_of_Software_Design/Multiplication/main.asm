section .data
    prompt1 db "Enter A: ", 0
    prompt2 db "Enter B: ", 0
    result_msg db "Product: ", 0

section .bss
    A resb 4
    B resb 4
    product resb 4

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

    ; Проверка на ноль
    test ebx, ebx
    jz .zero_case

    ; Умножение через сложение (битовая оптимизация)
    xor ecx, ecx  ; Счетчик суммы (итоговый результат)

.loop:
    test ebx, 1   ; Проверяем младший бит B
    jz .skip_add  ; Если ноль, пропускаем сложение
    add ecx, eax  ; Прибавляем A к результату

.skip_add:
    shl eax, 1    ; A *= 2 (сдвиг влево)
    shr ebx, 1    ; B /= 2 (сдвиг вправо)
    jnz .loop     ; Пока B ≠ 0, продолжаем

    ; Записываем результат
    mov [product], ecx
    jmp .print_result

.zero_case:
    mov dword [product], 0

.print_result:
    ; Вывод результата
    mov eax, 4
    mov ebx, 1
    mov ecx, result_msg
    mov edx, 9
    int 0x80

    mov eax, 4
    mov ebx, 1
    mov ecx, product
    mov edx, 4
    int 0x80

    ; Завершение программы
    mov eax, 1
    xor ebx, ebx
    int 0x80
