from Boyer_moore_search import boyer_moore_search

# Тесты для поиска подстроки в строке алгоритмом Боуэра-Мура

# Случай, когда игла пустая
assert boyer_moore_search("Hello world! I'm Bogdan", "") == None

# Случай, когда стог пустой
assert boyer_moore_search("", "Hello") == None

# Случай, когда стог и игла пустые
assert boyer_moore_search("", "") == None

# Случай, когда иглы нет в стоге
assert boyer_moore_search("Hello world! I'm Bogdan", "Hi") == None

# Случай, когда в стоге есть более одной иглы
assert boyer_moore_search("It's PythonPythonPythonPython", "Python") == 5
