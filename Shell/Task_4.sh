#!/bin/bash

# Проверяем, был ли передан каталог в качестве аргумента
if [ -z "$1" ]; then
  echo "Ошибка: не указан каталог."
  exit 1
fi

# Ищем файл или каталог с самым длинным именем
find "$1" -type f -or -type d | while read -r file; do
  basename "$file"
done | awk '{ if (length($0) > max) { max=length($0); name=$0 } } END { print name }'
