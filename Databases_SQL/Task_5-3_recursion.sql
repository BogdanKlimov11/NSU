-- Тест создания диапазона дат
SELECT daterange('2025-03-01', '2025-03-02', '[]') AS date_range;

-- Удаление таблицы travels, если она существует
DROP TABLE IF EXISTS travels CASCADE;

-- Создание таблицы для хранения поездок
CREATE TABLE travels (
    id           BIGINT GENERATED ALWAYS AS IDENTITY,
    travel_dates DATERANGE NOT NULL,
    CONSTRAINT travels_pk PRIMARY KEY (id),
    CONSTRAINT no_overlapping_dates EXCLUDE USING SPGIST (travel_dates WITH &&)
);

-- Вставка данных о поездках
INSERT INTO travels (travel_dates)
VALUES 
    (daterange('2025-03-02', '2025-03-02', '[]')),
    (daterange('2025-03-06', '2025-03-09', '[]')),
    (daterange('2025-03-11', '2025-03-12', '[]')),
    (daterange('2025-03-16', '2025-03-17', '[]')),
    (daterange('2025-03-25', '2025-03-27', '[]'));

-- Поиск свободных дат в заданном диапазоне
WITH RECURSIVE calendar AS (
    -- Начальная точка: полный диапазон дат
    SELECT 
        daterange('2025-03-01', '2025-04-01', '[]') AS left_range,
        daterange('2025-03-01', '2025-04-01', '[]') AS center_range,
        daterange('2025-03-01', '2025-04-01', '[]') AS right_range
    UNION
    -- Рекурсивное разбиение диапазонов на основе пересечений с поездками
    SELECT 
        CASE 
            WHEN travels.travel_dates && calendar.left_range 
            THEN daterange(lower(calendar.left_range), lower(travels.travel_dates * calendar.left_range), '[]')
            ELSE daterange(lower(calendar.right_range), lower(travels.travel_dates * calendar.right_range), '[]')
        END AS left_range,
        CASE 
            WHEN travels.travel_dates && calendar.left_range 
            THEN travels.travel_dates * calendar.left_range
            ELSE travels.travel_dates * calendar.right_range
        END AS center_range,
        CASE 
            WHEN travels.travel_dates && calendar.right_range 
            THEN daterange(upper(travels.travel_dates * calendar.right_range), upper(calendar.right_range), '[]')
            ELSE daterange(upper(travels.travel_dates * calendar.left_range), upper(calendar.left_range), '[]')
        END AS right_range
    FROM calendar
    JOIN travels ON 
        travels.travel_dates && daterange('2025-03-01', '2025-04-01', '[]') AND
        travels.travel_dates <> calendar.center_range AND (
            travels.travel_dates && calendar.left_range OR
            travels.travel_dates && calendar.right_range
        )
)
-- Выборка свободных диапазонов дат
SELECT available_dates
FROM (
    -- Свободные диапазоны из левой части
    SELECT a.left_range AS available_dates
    FROM calendar a
    LEFT JOIN calendar b 
        ON a.left_range <> b.left_range 
        AND a.left_range @> b.left_range
    GROUP BY a.left_range
    HAVING NOT bool_or(a.left_range @> b.left_range IS TRUE)
    UNION
    -- Свободные диапазоны из правой части
    SELECT a.right_range AS available_dates
    FROM calendar a
    LEFT JOIN calendar b 
        ON a.right_range <> b.right_range 
        AND a.right_range @> b.right_range
    GROUP BY a.right_range
    HAVING NOT bool_or(a.right_range @> b.right_range IS TRUE)
) free_dates
ORDER BY available_dates;
