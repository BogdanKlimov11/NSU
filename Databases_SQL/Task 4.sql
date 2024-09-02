--4-1

DROP TABLE IF EXISTS a, b;

CREATE TABLE a
(
    common INT,
    a_data VARCHAR(100)
);

CREATE TABLE b
(
    common INT,
    b_data VARCHAR(100)
);

INSERT INTO a (common, a_data)
VALUES (1, 'a_data1'),
       (2, 'a_data2');

INSERT INTO b (common, b_data)
VALUES (1, 'b_data1'),
       (3, 'b_data2'),
       (1, 'b_data3');

SELECT *
FROM a
         FULL OUTER JOIN b ON a.common = b.common
WHERE a.common IS NULL
   OR b.common IS NULL;

SELECT *
FROM a
         FULL OUTER JOIN b ON a.common = b.common;

SELECT *
FROM a
         INNER JOIN b ON a.common = b.common;

SELECT *
FROM a
         LEFT JOIN b ON a.common = b.common;

SELECT *
FROM a
         LEFT JOIN b ON a.common = b.common
WHERE b.common IS NULL;

SELECT *
FROM a
         RIGHT JOIN b ON a.common = b.common;

SELECT *
FROM a
         RIGHT JOIN b ON a.common = b.common
WHERE a.common IS NULL;


-- 4-2

SELECT sur_key, header
FROM tasks AS out
WHERE priority = (SELECT MAX(priority) FROM tasks AS int WHERE int.creator = out.creator);

SELECT b.sur_key, b.header
FROM tasks AS a,
     tasks AS b
WHERE a.creator = b.creator
GROUP BY b.sur_key, a.creator
HAVING max(a.priority) = b.priority;


-- 4-3

SELECT login
FROM users
WHERE login NOT IN (SELECT responsible FROM tasks WHERE responsible IS NOT NULL);

SELECT DISTINCT login
FROM users
         LEFT OUTER JOIN tasks AS t ON users.login = t.responsible
WHERE t.responsible IS NULL;

SELECT DISTINCT u.login
FROM users AS u
WHERE (SELECT DISTINCT responsible FROM tasks AS t WHERE t.responsible = u.login AND responsible IS NOT NULL
      ) IS NULL;


-- 4-4

SELECT responsible, creator
FROM tasks
WHERE responsible IS NOT NULL
UNION
SELECT responsible, creator
FROM tasks
WHERE responsible IS NOT NULL;


-- 4-5) origin and 2 synonyms

SELECT p.name, t.header
FROM tasks as t
         CROSS JOIN projects as p;

SELECT p.name, t.header
FROM tasks as t
         JOIN projects as p ON TRUE;

SELECT p.name, t.header
FROM tasks as t,
     projects as p;
