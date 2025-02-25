section .text
    global is_power_of_two  ; Экспортируем функцию

is_power_of_two:
    ; Вход: eax = число N
    ; Выход: eax = 1 (если степень двойки), 0 (иначе)
    
    test eax, eax       ; Проверяем, что N > 0
    jz not_power        ; Если N == 0, сразу выход
    
    mov ecx, eax        ; Сохраняем N
    dec ecx             ; ecx = N - 1
    and eax, ecx        ; eax = N & (N - 1)
    
    jz is_power         ; Если eax == 0, число - степень двойки
    
not_power:
    xor eax, eax        ; eax = 0 (ложь)
    ret
    
is_power:
    mov eax, 1          ; eax = 1 (истина)
    ret
