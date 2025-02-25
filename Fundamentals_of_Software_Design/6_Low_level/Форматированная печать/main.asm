section .data
    newline db 10, 0    ; Символ новой строки

section .text
    global my_printf   ; Экспортируем функцию

extern printf          ; Подключаем стандартный printf (для теста)

my_printf:
    ; Вход: 
    ;  - [esp+4]  = указатель на форматную строку
    ;  - [esp+8], [esp+12], ... = аргументы
    ; Выход: нет

    push ebp
    mov ebp, esp
    sub esp, 16         ; Буфер для вывода чисел

    mov esi, [ebp+8]    ; Загружаем адрес строки формата
    add ebp, 12         ; Устанавливаем ebp на первый аргумент

.loop:
    lodsb               ; Загружаем следующий символ в AL
    test al, al         ; Если конец строки (null terminator), выходим
    jz .done

    cmp al, '%'         ; Если не '%', просто печатаем символ
    jne .print_char

    lodsb               ; Загружаем следующий символ после '%'
    cmp al, '%'         ; %%
    je .print_char

    cmp al, 'd'         ; %d — число
    je .print_int

    cmp al, 's'         ; %s — строка
    je .print_str

    jmp .loop           ; Если неизвестный формат, пропускаем

.print_char:
    push eax            ; Сохраняем символ
    push 1              ; Дескриптор stdout
    push esp            ; Адрес символа
    push 1              ; Длина 1
    mov eax, 4          ; Системный вызов write
    int 0x80
    add esp, 16         ; Очищаем стек
    jmp .loop

.print_str:
    mov eax, [ebp]      ; Загружаем строку (адрес)
    add ebp, 4          ; Смещаемся к следующему аргументу

    push eax            ; Аргумент write (адрес)
    push eax            ; Аргумент для вычисления длины
    call strlen         ; Вычисляем длину строки
    push eax            ; Длина строки
    push 1              ; stdout
    mov eax, 4          ; Системный вызов write
    int 0x80
    add esp, 16
    jmp .loop

.print_int:
    mov eax, [ebp]      ; Загружаем число
    add ebp, 4          ; Смещаемся к следующему аргументу

    push eax            ; Передаём в функцию
    call itoa           ; Преобразуем в строку
    add esp, 4          ; Очищаем стек

    push eax            ; Аргумент write (адрес строки)
    call strlen         ; Вычисляем длину строки
    push eax            ; Длина строки
    push 1              ; stdout
    mov eax, 4          ; Системный вызов write
    int 0x80
    add esp, 16
    jmp .loop

.done:
    add esp, 16         ; Восстанавливаем стек
    pop ebp
    ret

; --- Вспомогательная функция: strlen ---
strlen:
    push ecx
    push edi
    mov edi, [esp+8]   ; Адрес строки
    mov ecx, -1
    xor al, al
    repne scasb
    not ecx
    dec ecx
    mov eax, ecx
    pop edi
    pop ecx
    ret

; --- Вспомогательная функция: itoa (int -> строка) ---
itoa:
    ; Вход: eax — число
    ; Выход: eax — адрес строки
    push ebx
    push ecx
    push edx
    push esi

    mov esi, esp        ; Буфер
    mov ecx, 10         ; Основание системы счисления
    xor ebx, ebx        ; Флаг знака

    test eax, eax
    jns .conv           ; Если положительное, пропустить знак

    neg eax             ; Обратный знак
    mov bl, 1           ; Установить флаг знака

.conv:
    xor edx, edx
    div ecx             ; Делим число на 10
    add dl, '0'         ; Преобразуем в ASCII
    dec esp
    mov [esp], dl       ; Записываем символ в стек
    test eax, eax
    jnz .conv           ; Повторяем, пока число не станет 0

    test bl, bl
    jz .done            ; Если число было положительное, завершить

    dec esp
    mov byte [esp], '-' ; Добавить знак минуса

.done:
    mov eax, esp        ; Возвращаем адрес строки
    pop esi
    pop edx
    pop ecx
    pop ebx
    ret
