-- 2-1
SELECT login, avg(tasks.priority) AS average
FROM users,
     tasks
WHERE tasks.responsible = users.login
GROUP BY login
ORDER BY average
LIMIT 3;

-- 2-2
SELECT concat(count(tasks.sur_key), ' - ', extract(MONTH FROM date_on), ' - ', login) AS stats
FROM users,
     tasks
WHERE tasks.responsible = users.login
  AND date_on IS NOT NULL
  AND extract(YEAR FROM date_on) = 2015
GROUP BY users.login, extract(MONTH from date_on);

-- 2-3
SELECT login executor, sum(a.estimate - a.spent) "-", sum(b.spent - b.estimate) "+"  -- как еще один раз к task
FROM users,
     tasks a,
     tasks b
WHERE users.login = a.responsible
  AND a.responsible = b.responsible
  AND a.estimate > a.spent
  AND b.spent > b.estimate
GROUP BY login;

SELECT login executor, sum(a.under) AS "-", sum(b.over) AS "+"
FROM users,
     (SELECT (estimate - spent) as under, responsible
      FROM tasks
      WHERE estimate > spent) AS a,

     (SELECT (spent - estimate) as over, responsible
      FROM tasks
      WHERE spent > estimate) AS b
WHERE users.login = a.responsible
  AND a.responsible = b.responsible
GROUP BY login;

-- 2-4
SELECT creator, responsible
FROM tasks
GROUP BY creator, responsible;

-- 2-5
SELECT login, length(login)
FROM users
ORDER BY length DESC
LIMIT 1;

-- 2-6
DROP TABLE IF EXISTS Ch, VCh;
CREATE TABLE Ch
(
  str VARCHAR(10485760)
);
CREATE TABLE VCh
(
  str CHAR(10485760)
); -- 1 MB
INSERT INTO Ch VALUES ('YSDA');
INSERT INTO VCh VALUES ('YSDA');

SELECT sum(pg_column_size(Ch.str)), sum(pg_column_size(VCh.str))
FROM Ch,
     VCh;

-- 2-7
SELECT login, max(priority)
FROM users,
     tasks
WHERE tasks.responsible = users.login
GROUP BY login, responsible;

-- 2-8
SELECT responsible, sum(estimate)
FROM tasks,
     (SELECT avg(estimate) FROM tasks) as a
WHERE estimate > a.avg
GROUP BY responsible, a.avg;

-- 2-9 -- Какой вью можно прокешировать?
CREATE VIEW task_counter AS
SELECT tasks.responsible,
       count(tasks.responsible) AS amount
FROM tasks
GROUP BY tasks.responsible;


CREATE VIEW task_complete AS
SELECT responsible,
       count(responsible) AS completed_task
FROM tasks
WHERE spent <= estimate
GROUP BY responsible;


CREATE VIEW task_delayed AS
SELECT responsible,
       count(responsible) AS delayed_task
FROM tasks
WHERE spent > estimate
GROUP BY responsible;
-- --------------
-- Не догнал, что хотят, кучу представлений или одно большое.
-- Если второе - то как?
-------------------------

--2-10

SELECT tasks.header, users.login
FROM users,
     tasks
WHERE users.login = tasks.creator;

----------------------------------

SELECT tasks.header, users.login
FROM (SELECT header, creator FROM tasks) AS tasks,
     (SELECT login FROM users) AS users
WHERE users.login = tasks.creator;

------------------------
SELECT tasks.header, (SELECT login FROM users WHERE login = tasks.creator)
FROM tasks;

SELECT header, creator FROM tasks
WHERE creator IN (SELECT login FROM users WHERE login = tasks.creator);
------------------------
