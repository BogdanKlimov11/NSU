# Задача «Удаление пробелов в начале и конце строки»

Реализовать функцию, убирающую пробельные символы в начале и конце переданной строки.

Не допускается использование дополнительных строк (дополнительной памяти).

Для определения пробельности символов использовать функцию `isspace()` из `ctype.h`.

```python
def strtrim(string):
   ...
```

```c
void strtrim(char *string) 
{
   /* ... */
}
```

Пример использования:

```python
buf = "   abc   "
print(f"<{buf}>")  # <   abc   >
trimmed = strtrim(buf)
print(f"<{trimmed}>")  # <abc>
```

```c
char buf[] = " abc ";
printf("<%s>\n", buf); // < abc >
strtrim(buf);
printf("<%s>\n", buf); // <abc>
```
