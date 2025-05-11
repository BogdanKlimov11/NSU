-- Удаление таблиц, если они существуют
DROP TABLE IF EXISTS table_a, table_b CASCADE;

-- Создание таблицы table_a
CREATE TABLE table_a (
    shared_id  BIGINT NOT NULL,
    data       VARCHAR(100),
    CONSTRAINT table_a_pk PRIMARY KEY (shared_id)
);

-- Создание таблицы table_b
CREATE TABLE table_b (
    shared_id  BIGINT NOT NULL,
    data       VARCHAR(100),
    CONSTRAINT table_b_pk PRIMARY KEY (shared_id)
);

-- Вставка тестовых данных в table_a
INSERT INTO table_a (shared_id, data)
VALUES 
    (1, 'a_data1'),
    (2, 'a_data2');

-- Вставка тестовых данных в table_b
INSERT INTO table_b (shared_id, data)
VALUES 
    (1, 'b_data1'),
    (3, 'b_data2'),
    (1, 'b_data3');

-- 4-1: Демонстрация различных типов JOIN

-- Запрос 1: Несоответствующие записи (только в table_a или table_b)
SELECT a.shared_id AS a_shared_id, a.data AS a_data, b.shared_id AS b_shared_id, b.data AS b_data
FROM table_a a
FULL OUTER JOIN table_b b ON a.shared_id = b.shared_id
WHERE a.shared_id IS NULL OR b.shared_id IS NULL;

-- Запрос 2: Полное соединение (все записи)
SELECT a.shared_id AS a_shared_id, a.data AS a_data, b.shared_id AS b_shared_id, b.data AS b_data
FROM table_a a
FULL OUTER JOIN table_b b ON a.shared_id = b.shared_id;

-- Запрос 3: Внутреннее соединение (общие записи)
SELECT a.shared_id AS a_shared_id, a.data AS a_data, b.shared_id AS b_shared_id, b.data AS b_data
FROM table_a a
INNER JOIN table_b b ON a.shared_id = b.shared_id;

-- Запрос 4: Левое соединение (все записи из table_a)
SELECT a.shared_id AS a_shared_id, a.data AS a_data, b.shared_id AS b_shared_id, b.data AS b_data
FROM table_a a
LEFT JOIN table_b b ON a.shared_id = b.shared_id;

-- Запрос 5: Записи только в table_a (без соответствия в table_b)
SELECT a.shared_id AS a_shared_id, a.data AS a_data, b.shared_id AS b_shared_id, b.data AS b_data
FROM table_a a
LEFT JOIN table_b b ON a.shared_id = b.shared_id
WHERE b.shared_id IS NULL;

-- Запрос 6: Правое соединение (все записи из table_b)
SELECT a.shared_id AS a_shared_id, a.data AS a_data, b.shared_id AS b_shared_id, b.data AS b_data
FROM table_a a
RIGHT JOIN table_b b ON a.shared_id = b.shared_id;

-- Запрос 7: Записи только в table_b (без соответствия в table_a)
SELECT a.shared_id AS a_shared_id, a.data AS a_data, b.shared_id AS b_shared_id, b.data AS b_data
FROM table_a a
RIGHT JOIN table_b b ON a.shared_id = b.shared_id
WHERE a.shared_id IS NULL;

-- 4-2: Поиск задач с максимальным приоритетом для каждого создателя

-- Вариант 1: Использование подзапроса
SELECT id, header
FROM tasks t1
WHERE priority = (
    SELECT MAX(priority)
    FROM tasks t2
    WHERE t2.creator = t1.creator
);

-- Вариант 2: Использование оконной функции (оптимизированный)
SELECT id, header
FROM (
    SELECT id, header, priority,
           MAX(priority) OVER (PARTITION BY creator) AS max_priority
    FROM tasks
) ranked
WHERE priority = max_priority;

-- 4-3: Поиск пользователей, не ответственных за задачи

-- Вариант 1: Использование LEFT JOIN (оптимизированный)
SELECT u.login
FROM users u
LEFT JOIN tasks t ON u.login = t.responsible
WHERE t.responsible IS NULL;

-- Вариант 2: Использование NOT EXISTS
SELECT u.login
FROM users u
WHERE NOT EXISTS (
    SELECT 1
    FROM tasks t
    WHERE t.responsible = u.login
);

-- 4-4: Объединение пар responsible и creator

SELECT responsible, creator
FROM tasks
WHERE responsible IS NOT NULL
UNION ALL
SELECT responsible, creator
FROM tasks
WHERE responsible IS NOT NULL;

-- 4-5: Демонстрация CROSS JOIN

-- Вариант 1: Явный CROSS JOIN
SELECT p.name AS project_name, t.header AS task_header
FROM tasks t
CROSS JOIN projects p;

-- Вариант 2: JOIN с условием TRUE
SELECT p.name AS project_name, t.header AS task_header
FROM tasks t
JOIN projects p ON TRUE;

-- Вариант 3: Устаревший синтаксис (для демонстрации)
SELECT p.name AS project_name, t.header AS task_header
FROM tasks t, projects p;
