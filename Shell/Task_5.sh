#!/bin/bash

# Проверяем, был ли передан каталог в качестве аргумента
if [ -z "$1" ]; then
  echo "Ошибка: не указан каталог."
  exit 1
fi

# Собираем длины всех файлов в каталогах
files=$(find "$1" -type f)
total_length=0
total_files=0
sum_of_squares=0

for file in $files; do
  length=$(wc -c < "$file")
  total_length=$((total_length + length))
  sum_of_squares=$((sum_of_squares + length * length))
  total_files=$((total_files + 1))
done

# Вычисляем среднюю длину и среднеквадратичное отклонение
if [ "$total_files" -ne 0 ]; then
  mean_length=$((total_length / total_files))
  variance=$(( (sum_of_squares / total_files) - (mean_length * mean_length) ))
  stddev=$(echo "scale=2; sqrt($variance)" | bc)
  echo "Средняя длина: $mean_length"
  echo "Среднеквадратичное отклонение: $stddev"
else
  echo "Нет регулярных файлов."
fi
