# Задача «Friends of Friends»

Моделируем работу движка социальной сети.

Дан граф френдования в формате `IDюзера IDфренда1 IDфренда2 …`:

```bash
vasya kolya petya sasha
sasha vasya cool123 petya
cool123 kolya
kolya cool123
```

Здесь у юзера `vasya` 3 френда и т.д. Примем, что `IDюзера` может
содержать цифры и латинские буквы обоих регистров.

Также дан список постов в социальной сети: `Время IDавтора Текст`:

```bash
2014-10-20T08:00:00 vasya Блин, сегодня опять понедельник :(
2014-10-20T08:05:00 cool123 Ура, понедельник!!!
2014-10-20T08:39:00 kolya Какой сегодня день?
```

(время в формате <a href="https://ru.wikipedia.org/wiki/ISO_8601">ISO8601</a>)

### Задача

Для заданного `userID` построить френдленту (все посты френдов, отсортированные
по времени в порядке убывания, т.е. самые свежие в начале), а также ленту
«friends of friends», т.е. посты френдов и френдов френдов, также отсортированные
по времени в порядке убывания. Посты не должны повторяться.

### Дополнение

Отдельным методом реализовать вычисление «количества рукопожатий» между двумя
юзерами (т.е. какой длины путь в графе френдования их разделяет).
