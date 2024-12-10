DROP TABLE IF EXISTS Users, Tasks, Projects;

CREATE TABLE Users
(
  name       VARCHAR(32) NOT NULL,
  login      VARCHAR(32),
  email      VARCHAR(32),
  department VARCHAR(16) CHECK (department in ('Production', 'Support', 'Accounting', 'Administration')),
  PRIMARY KEY (login)
);

CREATE TABLE Projects
(
  name        VARCHAR(32),
  description TEXT,
  date_on     DATE NOT NULL,
  date_off    DATE,
  PRIMARY KEY (name)
);

CREATE TABLE Tasks
(
  project     VARCHAR(32),
  header      VARCHAR(32)                                                               NOT NULL,
  priority    INTEGER                                                                   NOT NULL,
  description TEXT,
  status      VARCHAR(16) CHECK (status in ('new', 'reopened', 'closed', 'in process')) NOT NULL,
  estimate    INTEGER,
  spent       INTEGER,
  creator     VARCHAR(32),
  responsible VARCHAR(32),
  sur_key     SERIAL, -- increment : fixed
  date_on     DATE,
  FOREIGN KEY (project) REFERENCES Projects (name),
  FOREIGN KEY (creator) REFERENCES Users (login),
  FOREIGN KEY (responsible) REFERENCES Users (login),
  PRIMARY KEY (sur_key)
);

-- 2
INSERT INTO Users(name, login, email, department)
VALUES ('Касаткин Артем', 'a.kasatkin', 'a.kasatkin@ya.ru', 'Administration'),
       ('Петрова София', 's.petrova', 's.petrova@ya.ru', 'Accounting'),
       ('Дроздов Федр', 'f.drozdov', 'f.drozdov@ya.ru', 'Production'),
       ('Иванова Василина', 'v.ivanova', 'v.ivanova@ya.ru', 'Accounting'),
       ('Беркут Алексей', 'a.berkut', 'a.berkut@ya.ru', 'Support'),
       ('Белова Вера', 'v.belova', 'v.belova@ya.ru', 'Support'),
       ('Макенрой Алексей', 'a.makenroy', 'a.makenroy@ya.ru', 'Administration');

INSERT INTO Projects(name, date_on, date_off)
VALUES ('РТК', '2016/01/31', NULL),
       ('СС.Контент', '2015/02/23', '2016/12/31'),
       ('Демо-Сибирь', '2015/05/11', '2015/01/31'),
       ('МВД-Онлайн', '2015/05/22', '2016/01/31'),
       ('Поддержка', '2016/06/07', NULL);

INSERT INTO Tasks(project, header, priority, status, creator, responsible, date_on, estimate, spent)
VALUES ('Поддержка', 'Task1', 12, 'new', 'a.berkut', 's.petrova', NULL, 10, 15),
       ('Демо-Сибирь', 'Task2', 228, 'new', 'v.belova', 's.petrova', NULL, 52, 22),
       ('РТК', 'Task3', 1337, 'new', 'a.makenroy', 's.petrova', NULL, 12, 12),
       ('Демо-Сибирь', 'Task4', 10, 'new', 'v.ivanova', 'a.makenroy', NULL, 1, 100),
       ('МВД-Онлайн', 'Task5', 61, 'new', 'a.berkut', 'f.drozdov', NULL, 12, 22),
       ('Поддержка', 'Task6', 127, 'new', 'a.makenroy', 's.petrova', NULL, 12, 12),
       ('РТК', 'Task7', 19, 'new', 'a.makenroy', 'v.belova', NULL, 22, 35),
       ('Демо-Сибирь', 'Task8', 1, 'new', 'a.makenroy', 's.petrova', NULL, 94, 12),
       ('МВД-Онлайн', 'Task9', 1, 'new', 'v.ivanova', 'f.drozdov', NULL, 88, 24),
       ('Демо-Сибирь', 'Task10', 11, 'new', 'a.makenroy', 'a.kasatkin', '2015/1/1', 99, 2),
       ('РТК', 'Task11', 22, 'new', 'v.ivanova', 'a.berkut', '2016/4/1', NULL, NULL),
       ('Демо-Сибирь', 'Task12', 3, 'new', 'a.makenroy', 'a.makenroy', '2015/8/1', 66, 32),
       ('СС.Контент', 'Task13', 1, 'new', 'a.makenroy', 'a.kasatkin', '2015/8/2', 99, 2),
       ('СС.Контент', 'Task14', 20, 'new', 'a.makenroy', 'a.kasatkin', '2015/8/3', 22, 3),
       ('СС.Контент', 'Task15', 20, 'new', 'a.makenroy', NULL, '2015/12/3', NULL, NULL);

--3a
SELECT *
FROM tasks;

--3b
SELECT name, department
FROM Users;

--3c
SELECT login, email
FROM Users;

--3d
SELECT *
FROM tasks
WHERE priority > 50;

--3e
SELECT DISTINCT responsible
FROM tasks
WHERE responsible IS NOT NULL;

--3f
SELECT creator
FROM tasks
UNION
SELECT responsible
FROM tasks;

--3k
SELECT sur_key, header
FROM tasks
WHERE creator != 's.petrova'
  AND (responsible IN ('v.ivanova', 'a.makenroy', 'a.berkut'));
-- IN : fixed

--4
SELECT *
FROM tasks
WHERE responsible LIKE '%kasatkin%'
  AND date_on BETWEEN '2016/01/01%' AND '2016/01/03%';
-- wildcard : fixed

--5
SELECT t.sur_key, t.header, d.department
FROM tasks t,
     users d
WHERE t.responsible LIKE '%petrov%'
  AND t.creator = d.login
  AND d.department IN ('Production', 'Accounting', 'Administration');

--6
SELECT *
FROM tasks
WHERE responsible IS NULL;

UPDATE tasks
SET responsible = 's.petrova'
WHERE responsible IS NULL;

--7
DROP TABLE IF EXISTS tasks2;

CREATE TABLE tasks2 AS
SELECT *
FROM tasks;

-- 8
SELECT *
FROM users
WHERE name NOT LIKE '%a'
  AND login LIKE '%r%';
