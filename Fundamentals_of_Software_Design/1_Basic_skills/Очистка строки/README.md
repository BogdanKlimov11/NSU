# Задача «Очистка строки»

Реализовать функцию, очищающую переданную строку от излишних пробелов (если в строке встречается более одного пробела подряд, они должны быть заменены на один пробел).

Не допускается использование дополнительных строк (дополнительной памяти).

Для определения пробельности символов использовать функцию `isspace()` из `ctype.h`.

```python
def strclear(string):
  ...
```

```c
void strclear(char *string) 
{
   /* ... */
}
```

Пример использования:

```python
input_str = "ab    cd"
print(f"<{input_str}>")  # <ab    cd>
output_str = strclear(input_str)
print(f"<{output_str}>")  # <ab cd>
```

```c
char buf[] = "ab    cd";
printf("<%s>\n", buf); // <ab    cd>
strclear(buf);
printf("<%s>\n", buf); // <ab cd>
```
