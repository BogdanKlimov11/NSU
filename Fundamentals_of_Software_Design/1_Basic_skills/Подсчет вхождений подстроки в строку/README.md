# Задача «Подсчет вхождений подстроки в строку»

Реализовать функцию, подсчитывающую, сколько раз встречается подстрока в заданной строке.

```python
def substring_count(string, substring):
  ...
```

```c
int substring_count(char *string, char *substring)
{
    /* ... */
}
```

Примеры:

```python
print(substring_count("abcabc", "ab"))  # 2
print(substring_count("abcabcd", "d"))  # 1
print(substring_count("abcabcd", "q"))  # 0
print(substring_count("aaaaaa", "aa"))  # 5
```

```c
substring_count("abcabc", "ab"); // 2
substring_count("abcabcd", "d"); // 1
substring_count("abcabcd", "q"); // 0
substring_count("aaaaaa", "aa"); // 5
```
