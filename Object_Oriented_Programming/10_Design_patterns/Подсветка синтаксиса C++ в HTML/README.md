# Задача «Подсветка синтаксиса C++ в HTML»

### Задание

Используя <a href="https://ru.wikipedia.org/wiki/Декоратор_(шаблон_проектирования)">паттерн
декоратор</a>, реализовать набор классов, позволяющий разнообразным образом
переводить C++ код в <a href="https://ru.wikipedia.org/wiki/HTML">HTML</a>
страницы: с подсветкой <a href="https://en.cppreference.com/w/cpp/keyword">ключевых
слов</a> или <a href="https://habr.com/ru/articles/207952/">токенов</a>,
расстановкой номеров строк и пр.

### Описание

1. Реализовать класс, который принимает ссылку на текст в конструкторе, и
   имеет функцию `write(ostream & out)`, которая записывает этот текст в
   выходной поток.

2. Реализовать несколько классов-декораторов над вышеописанным классам

* **HTMLDecorator** — простое преобразование исходного кода в HTML файл.
  Для этого достаточно добавить открывающие и закрывающие тэги в начало
  и конец файла, а так же заменить спецсимволы: <a href="https://ru.wikipedia.org/wiki/Мнемоники_в_HTML">одинарные
  и двойные кавычки, знаки больше меньше и знак амперсанда</a>.

### Пример:

```html
<html>
  <body>
    <pre>
      # include &lt;iostream&gt;
      void main() {
          std::cout &lt;&lt; &quot;Hellow world!&quot;;
      }
    </pre>
  </body>
</html>
```

* **LineDecorator** — простое добавление номеров строк в код. При обработке
  текста необходимо учесть ситуации, когда строк в коде более $10$, $100$,
  $1000$, и т. п.

### Пример:

```html
<html>
  <body>
    <pre>
     1| # include &lt;iostream&gt;
     2| <font color=0000FF>void</font> main() {
     3|     std::cout &lt;&lt; &quot;Hellow world!&quot;;
     4| }
    </pre>
  </body>
</html>
```

* **СPPDecorator** — примитивная подсветка <a href="https://en.cppreference.com/w/cpp/keyword">ключевых
  слов C++</a>. Чтобы задать цвет текста в html можно использовать тег `font`
  с атрибутом `color` — `<font color=(цвет в hex представлении)>`. Цвет
  описывается в RGB представлении – по байту на компонент. То есть $000000$
  — черный, $FF0000$ — красный, $00FF00$ — зеленый, $0000FF$ — синий.

### Пример:

```html
<html>
  <body>
    <pre>
      # include &lt;iostream&gt;
      <font color=0000FF>void</font> main() {
          std::cout &lt;&lt; &quot;Hellow world!&quot;;
      }
    </pre>
  </body>
</html>
```

* **CodeReviewDecorator** — подсветка всех одинаковых лексем (всех одинаковых
  имен, не являющихся ключевыми словами C++) одинаковым цветом. Например,
  если в коде встречается переменная `flag`, то она везде будет подсвечена
  одинаковым цветом. Аналогично с именами функций. Цвет для лексемы выбирается
  случайно, с превалированием к тёмным оттенкам (предполагается белый фон).

### Пример:

```html
<html>
  <body>
    <pre>
      # include &lt;iostream&gt;
      void <font color=#0000FF>main</font>() {
         int <font color=#00FF00>flag</font> = 1;
         std::cout &lt;&lt; <font color=#00FF00>flag</font>;
      }
    </pre>
  </body>
</html>
```
