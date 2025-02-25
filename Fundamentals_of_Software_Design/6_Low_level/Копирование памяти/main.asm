section .text
    global memcpy

memcpy:
    ; Аргументы:
    ;   edi = dst (куда)
    ;   esi = src (откуда)
    ;   ecx = count (сколько)
    
    mov eax, edi       ; Если dst > src, делаем обратное копирование
    cmp eax, esi
    jae reverse_copy   ; Если dst >= src, идем на копирование с конца
    
forward_copy:
    cld               ; Копируем вперед (по умолчанию)
    test ecx, ecx
    jz done           ; Если count == 0, выходим
    
    rep movsb         ; Копируем count байт (ecx раз)
    jmp done

reverse_copy:
    std               ; Устанавливаем направление копирования назад
    add esi, ecx      ; src += count
    add edi, ecx      ; dst += count
    dec esi           ; src -= 1 (указываем на последний байт)
    dec edi           ; dst -= 1 (указываем на последний байт)
    
    rep movsb         ; Копируем count байт назад
    cld               ; Восстанавливаем стандартное направление

done:
    ret               ; Возвращаемся
