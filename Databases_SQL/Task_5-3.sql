-- Удаление таблицы events, если она существует
DROP TABLE IF EXISTS events CASCADE;

-- Создание таблицы для хранения событий
CREATE TABLE events (
    event_name  VARCHAR(30) NOT NULL,
    event_time  TIMESTAMP WITH TIME ZONE NOT NULL,
    CONSTRAINT events_pk PRIMARY KEY (event_name, event_time)
);

-- Вставка данных о событиях
INSERT INTO events (event_name, event_time)
VALUES 
    ('SMTH1', '2025-01-20 12:21:01+00'),
    ('SMTH2', '2025-10-01 15:07:37+00'),
    ('SMTH3', '2025-02-13 09:15:18+00'),
    ('SMTH4', '2025-11-07 11:43:21+00'),
    ('SMTH5', '2025-06-02 10:11:31+00'),
    ('SMTH5_2', '2025-06-02 10:12:31+00'),
    ('SMTH6', '2025-08-19 14:09:20+00');

-- Решение 1: Выборка уникальных дат для событий между 8:00 и 12:00
SELECT DISTINCT event_time::DATE AS event_date
FROM events
WHERE event_time::TIME WITH TIME ZONE BETWEEN '08:00:00' AND '12:00:00'
ORDER BY event_date;

-- Решение 2: Генерация последовательности дат на основе событий
WITH RECURSIVE calendar AS (
    -- Начальная точка: минимальная дата из событий
    SELECT MIN(event_time)::DATE AS calendar_date
    FROM events
    UNION
    -- Добавление последующих дат до максимальной границы
    SELECT (calendar_date + INTERVAL '1 day')::DATE
    FROM calendar
    WHERE calendar_date < '2025-12-31'
)
SELECT calendar_date
FROM calendar
WHERE calendar_date IN (
    SELECT event_time::DATE
    FROM events
)
ORDER BY calendar_date;

-- Решение 3: Генерация всех дат в диапазоне и фильтрация по событиям
WITH event_dates AS (
    SELECT DISTINCT event_time::DATE AS event_date
    FROM events
)
SELECT generate_series(
    (SELECT MIN(event_date) FROM event_dates),
    (SELECT MAX(event_date) FROM event_dates),
    INTERVAL '1 day'
)::DATE AS event_date
FROM event_dates
WHERE event_date IN (SELECT event_date FROM event_dates)
ORDER BY event_date;
