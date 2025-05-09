-- Удаление таблиц, если они существуют
DROP TABLE IF EXISTS Tasks;
DROP TABLE IF EXISTS Projects;
DROP TABLE IF EXISTS Users;

-- Создание таблицы Users
CREATE TABLE Users (
    name        VARCHAR(32) NOT NULL,
    login       VARCHAR(32) NOT NULL,
    email       VARCHAR(32),
    department  VARCHAR(16) CHECK (department IN ('Production', 'Support', 'Accounting', 'Administration')),
    PRIMARY KEY (login)
);

-- Создание таблицы Projects
CREATE TABLE Projects (
    name        VARCHAR(32) NOT NULL,
    description TEXT,
    date_on     DATE NOT NULL,
    date_off    DATE,
    PRIMARY KEY (name)
);

-- Создание таблицы Tasks
CREATE TABLE Tasks (
    sur_key     SERIAL,
    project     VARCHAR(32) NOT NULL,
    header      VARCHAR(32) NOT NULL,
    priority    INTEGER NOT NULL,
    description TEXT,
    status      VARCHAR(16) CHECK (status IN ('new', 'reopened', 'closed', 'in process')) NOT NULL,
    estimate    INTEGER,
    spent       INTEGER,
    creator     VARCHAR(32),
    responsible VARCHAR(32),
    date_on     DATE,
    FOREIGN KEY (project) REFERENCES Projects (name),
    FOREIGN KEY (creator) REFERENCES Users (login),
    FOREIGN KEY (responsible) REFERENCES Users (login),
    PRIMARY KEY (sur_key)
);

-- Вставка данных в таблицу Users
INSERT INTO Users (name, login, email, department) VALUES
    ('Касаткин Артем',      'a.kasatkin',  'a.kasatkin@ya.ru',  'Administration'),
    ('Петрова София',       's.petrova',   's.petrova@ya.ru',   'Accounting'),
    ('Дроздов Федр',        'f.drozdov',   'f.drozdov@ya.ru',   'Production'),
    ('Иванова Василина',    'v.ivanova',   'v.ivanova@ya.ru',   'Accounting'),
    ('Беркут Алексей',      'a.berkut',    'a.berkut@ya.ru',    'Support'),
    ('Белова Вера',         'v.belova',    'v.belova@ya.ru',    'Support'),
    ('Макенрой Алексей',    'a.makenroy',  'a.makenroy@ya.ru',  'Administration');

-- Вставка данных в таблицу Projects
INSERT INTO Projects (name, date_on, date_off) VALUES
    ('РТК',           '2016-01-31', NULL),
    ('СС.Контент',    '2015-02-23', '2016-12-31'),
    ('Демо-Сибирь',   '2015-05-11', '2015-01-31'),
    ('МВД-Онлайн',    '2015-05-22', '2016-01-31'),
    ('Поддержка',     '2016-06-07', NULL);

-- Вставка данных в таблицу Tasks
INSERT INTO Tasks (project, header, priority, status, creator, responsible, date_on, estimate, spent) VALUES
    ('Поддержка',     'Task1',   12,   'new', 'a.berkut',    's.petrova',   NULL,        10,  15),
    ('Демо-Сибирь',   'Task2',   228,  'new', 'v.belova',    's.petrova',   NULL,        52,  22),
    ('РТК',           'Task3',   1337, 'new', 'a.makenroy',  's.petrova',   NULL,        12,  12),
    ('Демо-Сибирь',   'Task4',   10,   'new', 'v.ivanova',   'a.makenroy',  NULL,        1,   100),
    ('МВД-Онлайн',    'Task5',   61,   'new', 'a.berkut',    'f.drozdov',   NULL,        12,  22),
    ('Поддержка',     'Task6',   127,  'new', 'a.makenroy',  's.petrova',   NULL,        12,  12),
    ('РТК',           'Task7',   19,   'new', 'a.makenroy',  'v.belova',    NULL,        22,  35),
    ('Демо-Сибирь',   'Task8',   1,    'new', 'a.makenroy',  's.petrova',   NULL,        94,  12),
    ('МВД-Онлайн',    'Task9',   1,    'new', 'v.ivanova',   'f.drozdov',   NULL,        88,  24),
    ('Демо-Сибирь',   'Task10',  11,   'new', 'a.makenroy',  'a.kasatkin',  '2015-01-01', 99,  2),
    ('РТК',           'Task11',  22,   'new', 'v.ivanova',   'a.berkut',    '2016-04-01', NULL, NULL),
    ('Демо-Сибирь',   'Task12',  3,    'new', 'a.makenroy',  'a.makenroy',  '2015-08-01', 66,  32),
    ('СС.Контент',    'Task13',  1,    'new', 'a.makenroy',  'a.kasatkin',  '2015-08-02', 99,  2),
    ('СС.Контент',    'Task14',  20,   'new', 'a.makenroy',  'a.kasatkin',  '2015-08-03', 22,  3),
    ('СС.Контент',    'Task15',  20,   'new', 'a.makenroy',  NULL,          '2015-12-03', NULL, NULL);

-- Выборка всех задач
SELECT *
FROM Tasks;

-- Выборка имени и отдела пользователей
SELECT name, department
FROM Users;

-- Выборка логина и email пользователей
SELECT login, email
FROM Users;

-- Выборка задач с приоритетом больше 50
SELECT *
FROM Tasks
WHERE priority > 50;

-- Выборка уникальных ответственных, исключая NULL
SELECT DISTINCT responsible
FROM Tasks
WHERE responsible IS NOT NULL;

-- Объединение создателей и ответственных
SELECT creator
FROM Tasks
UNION
SELECT responsible
FROM Tasks;

-- Выборка задач, где создатель не 's.petrova' и ответственный в списке
SELECT sur_key, header
FROM Tasks
WHERE creator != 's.petrova'
  AND responsible IN ('v.ivanova', 'a.makenroy', 'a.berkut');

-- Выборка задач с ответственным 'kasatkin' и датой в январе 2016
SELECT *
FROM Tasks
WHERE responsible LIKE '%kasatkin%'
  AND date_on BETWEEN '2016-01-01' AND '2016-01-03';

-- Выборка задач с ответственным 'petrov' и создателем из определённых отделов
SELECT t.sur_key, t.header, d.department
FROM Tasks t
JOIN Users d ON t.creator = d.login
WHERE t.responsible LIKE '%petrov%'
  AND d.department IN ('Production', 'Accounting', 'Administration');

-- Выборка и обновление задач без ответственного
SELECT *
FROM Tasks
WHERE responsible IS NULL;

UPDATE Tasks
SET responsible = 's.petrova'
WHERE responsible IS NULL;

-- Создание копии таблицы Tasks
DROP TABLE IF EXISTS tasks2;

CREATE TABLE tasks2 AS
SELECT *
FROM Tasks;

-- Выборка пользователей, чьё имя не заканчивается на 'a' и логин содержит 'r'
SELECT *
FROM Users
WHERE name NOT LIKE '%a'
  AND login LIKE '%r%';
