#!/bin/bash

# Перебираем всех пользователей из /etc/passwd
while IFS=: read -r login _ home _; do
  # Проверяем, существует ли домашний каталог
  if [ -d "$home" ]; then
    # Проверяем, заходил ли пользователь в систему (с помощью команды last)
    if last "$login" | grep -q "$login"; then
      echo "$login"
    fi
  fi
done < /etc/passwd
