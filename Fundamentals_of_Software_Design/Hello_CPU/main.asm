section .data
    message db 'Hello CPU!', 0   ; строка для вывода

section .text
    global _start                ; точка входа для линковщика

_start:
    ; Печать строки
    mov eax, 4                  ; syscall номер 4 (sys_write)
    mov ebx, 1                  ; дескриптор файла 1 (stdout)
    mov ecx, message            ; адрес строки
    mov edx, 12                 ; длина строки
    int 0x80                    ; вызов системного прерывания

    ; Завершение программы
    mov eax, 1                  ; syscall номер 1 (sys_exit)
    xor ebx, ebx                ; код возврата 0
    int 0x80                    ; вызов системного прерывания
