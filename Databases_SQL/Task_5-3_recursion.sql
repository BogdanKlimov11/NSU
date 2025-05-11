-- Тест создания диапазона дат
SELECT daterange('2025-03-01', '2025-03-02');

-- Удаление таблицы travels, если она существует
DROP TABLE IF EXISTS travels;

-- Создание таблицы travels с ограничением на пересечение дат
CREATE TABLE travels
(
    id           SERIAL PRIMARY KEY,
    travel_dates DATERANGE NOT NULL, -- Диапазон дат поездки
    EXCLUDE USING SPGIST (travel_dates WITH &&) -- Исключает пересекающиеся диапазоны дат
);

-- Вставка данных о поездках
INSERT INTO travels (travel_dates)
VALUES (daterange('2025-03-02', '2025-03-02', '[]')),
       (daterange('2025-03-06', '2025-03-09', '[]')),
       (daterange('2025-03-11', '2025-03-12', '[]')),
       (daterange('2025-03-16', '2025-03-17', '[]')),
       (daterange('2025-03-25', '2025-03-27', '[]'));

-- Рекурсивный запрос для определения свободных дат
WITH RECURSIVE calendar AS (
    -- Начальная точка: полный диапазон дат
    SELECT daterange('2025-03-01', '2025-04-01') AS left,
           daterange('2025-03-01', '2025-04-01') AS center,
           daterange('2025-03-01', '2025-04-01') AS right
    UNION
    -- Рекурсивное разбиение диапазонов на основе пересечений с поездками
    SELECT CASE travels.travel_dates && calendar.left
               WHEN TRUE THEN daterange(lower(calendar.left), lower(travels.travel_dates * calendar.left))
               ELSE daterange(lower(calendar.right), lower(travels.travel_dates * calendar.right))
               END AS left,
           CASE travels.travel_dates && calendar.left
               WHEN TRUE THEN travels.travel_dates * calendar.left
               ELSE travels.travel_dates * calendar.right
               END AS center,
           CASE travels.travel_dates && calendar.right
               WHEN TRUE THEN daterange(upper(travels.travel_dates * calendar.right), upper(calendar.right))
               ELSE daterange(upper(travels.travel_dates * calendar.left), upper(calendar.left))
               END AS right
    FROM calendar
             JOIN travels ON
            travels.travel_dates && daterange('2025-03-01', '2025-04-01') AND
            travels.travel_dates <> calendar.center AND (
                    travels.travel_dates && calendar.left OR
                    travels.travel_dates && calendar.right
                )
)
-- Выборка свободных диапазонов дат
SELECT *
FROM (
         -- Свободные диапазоны из левой части
         SELECT a.left AS available_dates
         FROM calendar a
                  LEFT OUTER JOIN calendar b ON
                 a.left <> b.left AND
                 a.left @> b.left
         GROUP BY a.left
         HAVING NOT bool_or(COALESCE(a.left @> b.left, FALSE))
         UNION
         -- Свободные диапазоны из правой части
         SELECT a.right AS available_dates
         FROM calendar a
                  LEFT OUTER JOIN calendar b ON
                 a.right <> b.right AND
                 a.right @> b.right
         GROUP BY a.right
         HAVING NOT bool_or(COALESCE(a.right @> b.right, FALSE))
     ) a
ORDER BY available_dates;
