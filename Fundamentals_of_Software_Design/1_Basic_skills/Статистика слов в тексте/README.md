# Задача «Статистика слов в тексте»

Реализовать подсчет статистики встречаемости слов в файле.

`wordstat infile outfile` — подсчет статистики для `infile`, результат записывается в `outfile`.

Пример файла со статистикой:

```
во 2
саду-ли 1
огороде 1
```

Что считать словом?

1. Отдельным словом считать непрерывную последовательность непробельных символов. Пример: строка *"Мама мыла раму"* содержит $3$ слова - *"мама"*, *"мыла"*, *"раму"*.

2. Знаки пунктуации в начале и в конце слова отбрасываются, знаки пунктуации в середине слова считаются частью слова. Пример: строка: *" —Ты кто такой? — сказал кто-то."* содержит $5$ слов - *"Ты"*, *"кто"*, *"такой"*, *"сказал"*, *"кто-то"*.

Для реализации использовать либо <a href="https://opk.afti.ru/tasks/prefiksnoe-derevo-bor">бор</a>, либо <a href="https://opk.afti.ru/tasks/hesh-tablitsa-na-spiskah-kolliziy">хэш-таблицу</a>.
