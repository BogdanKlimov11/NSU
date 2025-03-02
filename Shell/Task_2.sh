#!/bin/bash

# Перебираем всех пользователей из /etc/passwd
while IFS=: read -r login _ _ _; do
  # Проверяем, есть ли процессы, запущенные этим пользователем
  if pgrep -u "$login" > /dev/null; then
    # Проверяем, не заходил ли пользователь в систему
    if ! last "$login" | grep -q "$login"; then
      echo "$login"
    fi
  fi
done < /etc/passwd
