from Kmp_search import kmp_search

# Тесты для поиска подстроки в строке алгоритмом Кнута-Морриса-Пратта

# Случай, когда игла пустая
assert kmp_search("Hello world! I'm Bogdan", "") == None

# Случай, когда стог пустой
assert kmp_search("", "Hello") == None

# Случай, когда стог и игла пустые
assert kmp_search("", "") == None

# Случай, когда иглы нет в стоге
assert kmp_search("Hello world! I'm Bogdan", "Hi") == None

# Случай, когда в стоге есть более одной иглы
assert kmp_search("It's PythonPythonPythonPython", "Python") == 5
