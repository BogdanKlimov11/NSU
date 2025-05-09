-- Выборка средней приоритетности задач по ответственным (топ-3)
SELECT u.login, 
       AVG(t.priority) AS average
FROM Users u
JOIN Tasks t ON t.responsible = u.login
GROUP BY u.login
ORDER BY average
LIMIT 3;

-- Статистика задач по месяцам 2015 года для ответственных
SELECT CONCAT(COUNT(t.sur_key), ' - ', EXTRACT(MONTH FROM t.date_on), ' - ', u.login) AS stats
FROM Users u
JOIN Tasks t ON t.responsible = u.login
WHERE t.date_on IS NOT NULL
  AND EXTRACT(YEAR FROM t.date_on) = 2015
GROUP BY u.login, EXTRACT(MONTH FROM t.date_on);

-- Выборка перерасхода и недооценки времени по ответственным (вариант 1)
SELECT u.login AS executor, 
       SUM(a.estimate - a.spent) AS under_estimation, 
       SUM(b.spent - b.estimate) AS over_estimation
FROM Users u
JOIN Tasks a ON u.login = a.responsible
JOIN Tasks b ON a.responsible = b.responsible
WHERE a.estimate > a.spent
  AND b.spent > b.estimate
GROUP BY u.login;

-- Выборка перерасхода и недооценки времени по ответственным (вариант 2)
SELECT u.login AS executor, 
       SUM(a.under) AS under_estimation, 
       SUM(b.over) AS over_estimation
FROM Users u
JOIN (
    SELECT (estimate - spent) AS under, responsible
    FROM Tasks
    WHERE estimate > spent
) a ON u.login = a.responsible
JOIN (
    SELECT (spent - estimate) AS over, responsible
    FROM Tasks
    WHERE spent > estimate
) b ON a.responsible = b.responsible
GROUP BY u.login;

-- Выборка пар создатель-ответственный
SELECT creator, responsible
FROM Tasks
GROUP BY creator, responsible;

-- Пользователь с самым длинным логином
SELECT login, LENGTH(login) AS length
FROM Users
ORDER BY length DESC
LIMIT 1;

-- Создание таблиц для сравнения размера строк
DROP TABLE IF EXISTS Ch, VCh;

CREATE TABLE Ch (
    str VARCHAR(10485760)
);

CREATE TABLE VCh (
    str CHAR(10485760)
);

-- Вставка тестовых данных
INSERT INTO Ch VALUES ('YSDA');
INSERT INTO VCh VALUES ('YSDA');

-- Сравнение размера строк в таблицах
SELECT SUM(pg_column_size(Ch.str)) AS varchar_size, 
       SUM(pg_column_size(VCh.str)) AS char_size
FROM Ch, VCh;

-- Максимальный приоритет задач по ответственным
SELECT u.login, 
       MAX(t.priority) AS max_priority
FROM Users u
JOIN Tasks t ON t.responsible = u.login
GROUP BY u.login;

-- Сумма оценок задач выше среднего по ответственным
SELECT t.responsible, 
       SUM(t.estimate) AS total_estimate
FROM Tasks t
CROSS JOIN (SELECT AVG(estimate) AS avg FROM Tasks) a
WHERE t.estimate > a.avg
GROUP BY t.responsible;

-- Создание представлений для статистики задач
CREATE VIEW task_counter AS
SELECT responsible, 
       COUNT(responsible) AS amount
FROM Tasks
GROUP BY responsible;

CREATE VIEW task_complete AS
SELECT responsible, 
       COUNT(responsible) AS completed_task
FROM Tasks
WHERE spent <= estimate
GROUP BY responsible;

CREATE VIEW task_delayed AS
SELECT responsible, 
       COUNT(responsible) AS delayed_task
FROM Tasks
WHERE spent > estimate
GROUP BY responsible;

-- Объединённое представление для всей статистики
CREATE VIEW task_stats AS
SELECT t.responsible, 
       COALESCE(COUNT(t.responsible), 0) AS total_tasks,
       COALESCE(SUM(CASE WHEN t.spent <= t.estimate THEN 1 ELSE 0 END), 0) AS completed_tasks,
       COALESCE(SUM(CASE WHEN t.spent > t.estimate THEN 1 ELSE 0 END), 0) AS delayed_tasks
FROM Tasks t
GROUP BY t.responsible;

-- Различные варианты выборки заголовков задач и их создателей
-- Вариант 1: Простое соединение
SELECT t.header, u.login
FROM Users u
JOIN Tasks t ON u.login = t.creator;

-- Вариант 2: С подзапросами
SELECT t.header, u.login
FROM (SELECT header, creator FROM Tasks) t
JOIN (SELECT login FROM Users) u ON u.login = t.creator;

-- Вариант 3: С подзапросом в SELECT
SELECT t.header, 
       (SELECT login FROM Users u WHERE u.login = t.creator) AS login
FROM Tasks t;

-- Вариант 4: С IN и подзапросом
SELECT header, creator
FROM Tasks
WHERE creator IN (SELECT login FROM Users WHERE login = creator);
