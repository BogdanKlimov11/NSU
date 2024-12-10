SELECT daterange('2018-03-01', '2018-03-02');

DROP TABLE IF EXISTS travels;

CREATE TABLE travels
(
    id           serial PRIMARY KEY,
    travel_dates daterange NOT NULL,
    EXCLUDE USING spgist (travel_dates WITH &&)
    -- This is an exclusion constraint, which uses a condition
    -- to prevent duplicate rows from existing in a table. In this case,
    -- we specified that we don't want two rows to have overlapping (&&)
    -- travel_dates, thus we should never double-book our travel
);

INSERT INTO travels (travel_dates)
VALUES (daterange('2018-03-02', '2018-03-02', '[]')),
       (daterange('2018-03-06', '2018-03-09', '[]')),
       (daterange('2018-03-11', '2018-03-12', '[]')),
       (daterange('2018-03-16', '2018-03-17', '[]')),
       (daterange('2018-03-25', '2018-03-27', '[]'));

WITH RECURSIVE calendar AS (
    SELECT daterange('2018-03-01', '2018-04-01') AS left,
           daterange('2018-03-01', '2018-04-01') AS center,
           daterange('2018-03-01', '2018-04-01') AS right
    UNION
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
            travels.travel_dates && daterange('2018-03-01', '2018-04-01') AND
            travels.travel_dates <> calendar.center AND (
                    travels.travel_dates && calendar.left OR
                    travels.travel_dates && calendar.right
                )
)
SELECT *
FROM (
         SELECT a.left AS available_dates
         FROM calendar a
                  LEFT OUTER JOIN calendar b ON
                 a.left <> b.left AND
                 a.left @> b.left
         GROUP BY a.left
         HAVING NOT bool_or(COALESCE(a.left @> b.left, FALSE))
         UNION
         SELECT a.right AS available_dates
         FROM calendar a
                  LEFT OUTER JOIN calendar b ON
                 a.right <> b.right AND
                 a.right @> b.right
         GROUP BY a.right
         HAVING NOT bool_or(COALESCE(a.right @> b.right, FALSE))
     ) a
ORDER BY available_dates;

SELECT daterange('2018-03-01', '2018-03-02');

DROP TABLE IF EXISTS travels;

CREATE TABLE travels
(
    id           serial PRIMARY KEY,
    travel_dates daterange NOT NULL,
    EXCLUDE USING spgist (travel_dates WITH &&)
    -- This is an exclusion constraint, which uses a condition
    -- to prevent duplicate rows from existing in a table. In this case,
    -- we specified that we don't want two rows to have overlapping (&&)
    -- travel_dates, thus we should never double-book our travel
);

INSERT INTO travels (travel_dates)
VALUES (daterange('2018-03-02', '2018-03-02', '[]')),
       (daterange('2018-03-06', '2018-03-09', '[]')),
       (daterange('2018-03-11', '2018-03-12', '[]')),
       (daterange('2018-03-16', '2018-03-17', '[]')),
       (daterange('2018-03-25', '2018-03-27', '[]'));

WITH RECURSIVE calendar AS (
    SELECT daterange('2018-03-01', '2018-04-01') AS left,
           daterange('2018-03-01', '2018-04-01') AS center,
           daterange('2018-03-01', '2018-04-01') AS right
    UNION
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
            travels.travel_dates && daterange('2018-03-01', '2018-04-01') AND
            travels.travel_dates <> calendar.center AND (
                    travels.travel_dates && calendar.left OR
                    travels.travel_dates && calendar.right
                )
)
SELECT *
FROM (
         SELECT a.left AS available_dates
         FROM calendar a
                  LEFT OUTER JOIN calendar b ON
                 a.left <> b.left AND
                 a.left @> b.left
         GROUP BY a.left
         HAVING NOT bool_or(COALESCE(a.left @> b.left, FALSE))
         UNION
         SELECT a.right AS available_dates
         FROM calendar a
                  LEFT OUTER JOIN calendar b ON
                 a.right <> b.right AND
                 a.right @> b.right
         GROUP BY a.right
         HAVING NOT bool_or(COALESCE(a.right @> b.right, FALSE))
     ) a
ORDER BY available_dates;
