# Задача «base64 кодирование»

Реализовать кодирование/декодирование данных по алгоритму base64.

base64 — обратимое кодирование, которое переводит все символы 8-битной кодовой таблицы в символы, гарантированно сохраняющиеся при передаче данных в любых сетях и между любыми устройствами.

В основе алгоритма лежит перевод трех 8-битных байтов в четыре 6-битных значения и запись этих 6-битных данных в виде текста (<a href="https://ru.wikipedia.org/wiki/ASCII">ASCII символы</a>).

Более подробное описание алгоритма на сайте <a href="https://ru.wikipedia.org/wiki/Base64">Wikipedia</a>.

Режим работы и файлы задаются в командной строке:

* `base64 -e infile outfile` — кодирование `infile`, запись результата в `outfile`.

* `base64 -d infile outfile` — декодирование `infile`, запись результата в `outfile`.

При кодировании программа должна вставлять переводы строк, разбивая получающийся текст на строки длинной не более 72 символов.

История <a href="https://habr.com/ru/articles/88077/">base64</a>.
