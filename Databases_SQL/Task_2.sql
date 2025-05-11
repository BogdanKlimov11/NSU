-- Выборка средней приоритетности задач по ответственным (топ-3)
SELECT u.login, 
       AVG(t.priority)::NUMERIC(5, 2) AS average_priority
FROM users u
JOIN tasks t ON t.responsible = u.login
GROUP BY u.login
ORDER BY average_priority
LIMIT 3;

-- Статистика задач по месяцам 2025 года для ответственных
SELECT CONCAT(COUNT(t.id), ' - ', EXTRACT(MONTH FROM t.date_on), ' - ', u.login) AS task_stats
FROM users u
JOIN tasks t ON t.responsible = u.login
WHERE t.date_on IS NOT NULL
  AND EXTRACT(YEAR FROM t.date_on) = 2025
GROUP BY u.login, EXTRACT(MONTH FROM t.date_on);

-- Выборка перерасхода и недооценки времени по ответственным (вариант 1: с использованием CTE)
WITH under_estimates AS (
    SELECT responsible, (estimate - spent) AS under_estimation
    FROM tasks
    WHERE estimate > spent
),
over_estimates AS (
    SELECT responsible, (spent - estimate) AS over_estimation
    FROM tasks
    WHERE spent > estimate
)
SELECT u.login AS executor, 
       COALESCE(SUM(ue.under_estimation), 0) AS under_estimation,
       COALESCE(SUM(oe.over_estimation), 0) AS over_estimation
FROM users u
LEFT JOIN under_estimates ue ON u.login = ue.responsible
LEFT JOIN over_estimates oe ON u.login = oe.responsible
GROUP BY u.login;

-- Выборка перерасхода и недооценки времени по ответственным (вариант 2: с подзапросами)
SELECT u.login AS executor, 
       COALESCE(SUM(a.under_estimation), 0) AS under_estimation,
       COALESCE(SUM(b.over_estimation), 0) AS over_estimation
FROM users u
LEFT JOIN (
    SELECT responsible, (estimate - spent) AS under_estimation
    FROM tasks
    WHERE estimate > spent
) a ON u.login = a.responsible
LEFT JOIN (
    SELECT responsible, (spent - estimate) AS over_estimation
    FROM tasks
    WHERE spent > estimate
) b ON u.login = b.responsible
GROUP BY u.login;

-- Выборка пар создатель-ответственный
SELECT creator, responsible
FROM tasks
WHERE creator IS NOT NULL AND responsible IS NOT NULL
GROUP BY creator, responsible;

-- Пользователь с самым длинным логином
SELECT login, 
       LENGTH(login) AS login_length
FROM users
ORDER BY login_length DESC
LIMIT 1;

-- Создание таблиц для сравнения размера строк
DROP TABLE IF EXISTS varchar_table, char_table CASCADE;

-- Создание таблицы с VARCHAR
CREATE TABLE varchar_table (
    data VARCHAR(10485760) NOT NULL,
    CONSTRAINT varchar_table_pk PRIMARY KEY (data)
);

-- Создание таблицы с CHAR
CREATE TABLE char_table (
    data CHAR(10485760) NOT NULL,
    CONSTRAINT char_table_pk PRIMARY KEY (data)
);

-- Вставка тестовых данных
INSERT INTO varchar_table (data) VALUES ('YSDA');
INSERT INTO char_table (data) VALUES ('YSDA');

-- Сравнение размера строк в таблицах
SELECT SUM(pg_column_size(vt.data)) AS varchar_size, 
       SUM(pg_column_size(ct.data)) AS char_size
FROM varchar_table vt
CROSS JOIN char_table ct;

-- Максимальный приоритет задач по ответственным
SELECT u.login, 
       MAX(t.priority) AS max_priority
FROM users u
JOIN tasks t ON t.responsible = u.login
GROUP BY u.login;

-- Сумма оценок задач выше среднего по ответственным
SELECT t.responsible, 
       SUM(t.estimate) AS total_estimate
FROM tasks t
CROSS JOIN (SELECT AVG(estimate) AS avg_estimate FROM tasks) avg
WHERE t.estimate > avg.avg_estimate
GROUP BY t.responsible;

-- Создание представлений для статистики задач

-- Удаление представлений, если они существуют
DROP VIEW IF EXISTS task_counter, task_complete, task_delayed, task_stats CASCADE;

-- Представление: общее количество задач по ответственным
CREATE VIEW task_counter AS
SELECT responsible, 
       COUNT(*) AS task_count
FROM tasks
WHERE responsible IS NOT NULL
GROUP BY responsible;

-- Представление: завершённые задачи (spent <= estimate)
CREATE VIEW task_complete AS
SELECT responsible, 
       COUNT(*) AS completed_tasks
FROM tasks
WHERE spent <= estimate AND responsible IS NOT NULL
GROUP BY responsible;

-- Представление: просроченные задачи (spent > estimate)
CREATE VIEW task_delayed AS
SELECT responsible, 
       COUNT(*) AS delayed_tasks
FROM tasks
WHERE spent > estimate AND responsible IS NOT NULL
GROUP BY responsible;

-- Объединённое представление для всей статистики
CREATE VIEW task_stats AS
SELECT t.responsible, 
       COUNT(*) AS total_tasks,
       SUM(CASE WHEN t.spent <= t.estimate THEN 1 ELSE 0 END) AS completed_tasks,
       SUM(CASE WHEN t.spent > t.estimate THEN 1 ELSE 0 END) AS delayed_tasks
FROM tasks t
WHERE t.responsible IS NOT NULL
GROUP BY t.responsible;

-- Варианты выборки заголовков задач и их создателей

-- Вариант 1: Простое соединение
SELECT t.header AS task_header, 
       u.login AS creator
FROM users u
JOIN tasks t ON u.login = t.creator;

-- Вариант 2: С подзапросами
SELECT t.header AS task_header, 
       u.login AS creator
FROM (SELECT header, creator FROM tasks WHERE creator IS NOT NULL) t
JOIN (SELECT login FROM users) u ON u.login = t.creator;

-- Вариант 3: С подзапросом в SELECT
SELECT t.header AS task_header, 
       (SELECT login FROM users u WHERE u.login = t.creator) AS creator
FROM tasks t
WHERE t.creator IS NOT NULL;

-- Вариант 4: С EXISTS (оптимизированный IN)
SELECT t.header AS task_header, 
       t.creator
FROM tasks t
WHERE EXISTS (
    SELECT 1 FROM users u WHERE u.login = t.creator
);
