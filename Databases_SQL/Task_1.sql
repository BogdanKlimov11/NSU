-- Удаление таблиц, если они существуют
DROP TABLE IF EXISTS tasks, projects, users CASCADE;

-- Создание таблицы users
CREATE TABLE users (
    name        VARCHAR(32) NOT NULL,
    login       VARCHAR(32) NOT NULL,
    email       VARCHAR(32),
    department  VARCHAR(16) CHECK (department IN ('Production', 'Support', 'Accounting', 'Administration')),
    CONSTRAINT users_pk PRIMARY KEY (login)
);

-- Создание таблицы projects
CREATE TABLE projects (
    name        VARCHAR(32) NOT NULL,
    description TEXT,
    date_on     DATE NOT NULL,
    date_off    DATE,
    CONSTRAINT projects_pk PRIMARY KEY (name)
);

-- Создание таблицы tasks
CREATE TABLE tasks (
    id          BIGINT GENERATED ALWAYS AS IDENTITY,
    project     VARCHAR(32) NOT NULL,
    header      VARCHAR(32) NOT NULL,
    priority    INTEGER NOT NULL,
    description TEXT,
    status      VARCHAR(16) CHECK (status IN ('new', 'reopened', 'closed', 'in_process')) NOT NULL,
    estimate    INTEGER,
    spent       INTEGER,
    creator     VARCHAR(32),
    responsible VARCHAR(32),
    date_on     DATE,
    CONSTRAINT tasks_pk PRIMARY KEY (id),
    CONSTRAINT tasks_project_fk FOREIGN KEY (project) REFERENCES projects (name),
    CONSTRAINT tasks_creator_fk FOREIGN KEY (creator) REFERENCES users (login),
    CONSTRAINT tasks_responsible_fk FOREIGN KEY (responsible) REFERENCES users (login)
);

-- Вставка данных в таблицу users
INSERT INTO users (name, login, email, department)
VALUES 
    ('Касаткин Артем',      'a_kasatkin',  'a.kasatkin@ya.ru',  'Administration'),
    ('Петрова София',       's_petrova',   's.petrova@ya.ru',   'Accounting'),
    ('Дроздов Федр',        'f_drozdov',   'f.drozdov@ya.ru',   'Production'),
    ('Иванова Василина',    'v_ivanova',   'v.ivanova@ya.ru',   'Accounting'),
    ('Беркут Алексей',      'a_berkut',    'a.berkut@ya.ru',    'Support'),
    ('Белова Вера',         'v_belova',    'v.belova@ya.ru',    'Support'),
    ('Макенрой Алексей',    'a_makenroy',  'a.makenroy@ya.ru',  'Administration');

-- Вставка данных в таблицу projects
INSERT INTO projects (name, date_on, date_off)
VALUES 
    ('РТК',           '2026-01-31', NULL),
    ('СС_Контент',    '2025-02-23', '2026-12-31'),
    ('Демо_Сибирь',   '2025-05-11', '2026-01-31'),
    ('МВД_Онлайн',    '2025-05-22', '2026-01-31'),
    ('Поддержка',     '2026-06-07', NULL);

-- Вставка данных в таблицу tasks
INSERT INTO tasks (project, header, priority, status, creator, responsible, date_on, estimate, spent)
VALUES 
    ('Поддержка',     'Task1',   12,   'new', 'a_berkut',    's_petrova',   NULL,         10,  15),
    ('Демо_Сибирь',   'Task2',   228,  'new', 'v_belova',    's_petrova',   NULL,         52,  22),
    ('РТК',           'Task3',   1337, 'new', 'a_makenroy',  's_petrova',   NULL,         12,  12),
    ('Демо_Сибирь',   'Task4',   10,   'new', 'v_ivanova',   'a_makenroy',  NULL,         1,   100),
    ('МВД_Онлайн',    'Task5',   61,   'new', 'a_berkut',    'f_drozdov',   NULL,         12,  22),
    ('Поддержка',     'Task6',   127,  'new', 'a_makenroy',  's_petrova',   NULL,         12,  12),
    ('РТК',           'Task7',   19,   'new', 'a_makenroy',  'v_belova',    NULL,         22,  35),
    ('Демо_Сибирь',   'Task8',   1,    'new', 'a_makenroy',  's_petrova',   NULL,         94,  12),
    ('МВД_Онлайн',    'Task9',   1,    'new', 'v_ivanova',   'f_drozdov',   NULL,         88,  24),
    ('Демо_Сибирь',   'Task10',  11,   'new', 'a_makenroy',  'a_kasatkin',  '2025-01-01', 99,  2),
    ('РТК',           'Task11',  22,   'new', 'v_ivanova',   'a_berkut',    '2026-04-01', NULL, NULL),
    ('Демо_Сибирь',   'Task12',  3,    'new', 'a_makenroy',  'a_makenroy',  '2025-08-01', 66,  32),
    ('СС_Контент',    'Task13',  1,    'new', 'a_makenroy',  'a_kasatkin',  '2025-08-02', 99,  2),
    ('СС_Контент',    'Task14',  20,   'new', 'a_makenroy',  'a_kasatkin',  '2025-08-03', 22,  3),
    ('СС_Контент',    'Task15',  20,   'new', 'a_makenroy',  NULL,          '2025-12-03', NULL, NULL);

-- Выборка всех задач
SELECT id, project, header, priority, status, creator, responsible, date_on, estimate, spent
FROM tasks;

-- Выборка имени и отдела пользователей
SELECT name, department
FROM users;

-- Выборка логина и email пользователей
SELECT login, email
FROM users;

-- Выборка задач с приоритетом больше 50
SELECT id, project, header, priority, status, creator, responsible, date_on, estimate, spent
FROM tasks
WHERE priority > 50;

-- Выборка уникальных ответственных, исключая NULL
SELECT DISTINCT responsible
FROM tasks
WHERE responsible IS NOT NULL
ORDER BY responsible;

-- Объединение создателей и ответственных
SELECT creator AS user_login
FROM tasks
WHERE creator IS NOT NULL
UNION ALL
SELECT responsible
FROM tasks
WHERE responsible IS NOT NULL
ORDER BY user_login;

-- Выборка задач, где создатель не 's_petrova' и ответственный в списке
SELECT id, header
FROM tasks
WHERE creator != 's_petrova'
  AND responsible IN ('v_ivanova', 'a_makenroy', 'a_berkut');

-- Выборка задач с ответственным 'a_kasatkin' и датой в январе 2026
SELECT id, project, header, priority, status, creator, responsible, date_on, estimate, spent
FROM tasks
WHERE responsible = 'a_kasatkin'
  AND date_on BETWEEN '2026-01-01' AND '2026-01-31';

-- Выборка задач с ответственным 's_petrova' и создателем из отделов
SELECT t.id, t.header, u.department
FROM tasks t
JOIN users u ON t.creator = u.login
WHERE t.responsible = 's_petrova'
  AND u.department IN ('Production', 'Accounting', 'Administration');

-- Выборка и обновление задач без ответственного
SELECT id, project, header, priority, status, creator, responsible, date_on, estimate, spent
FROM tasks
WHERE responsible IS NULL;

UPDATE tasks
SET responsible = 's_petrova'
WHERE responsible IS NULL;

-- Создание копии таблицы tasks
DROP TABLE IF EXISTS tasks_copy CASCADE;

CREATE TABLE tasks_copy (
    id          BIGINT NOT NULL,
    project     VARCHAR(32) NOT NULL,
    header      VARCHAR(32) NOT NULL,
    priority    INTEGER NOT NULL,
    description TEXT,
    status      VARCHAR(16) CHECK (status IN ('new', 'reopened', 'closed', 'in_process')) NOT NULL,
    estimate    INTEGER,
    spent       INTEGER,
    creator     VARCHAR(32),
    responsible VARCHAR(32),
    date_on     DATE,
    CONSTRAINT tasks_copy_pk PRIMARY KEY (id),
    CONSTRAINT tasks_copy_project_fk FOREIGN KEY (project) REFERENCES projects (name),
    CONSTRAINT tasks_copy_creator_fk FOREIGN KEY (creator) REFERENCES users (login),
    CONSTRAINT tasks_copy_responsible_fk FOREIGN KEY (responsible) REFERENCES users (login)
);

INSERT INTO tasks_copy
SELECT * FROM tasks;

-- Выборка пользователей, чьё имя не заканчивается на 'a' и логин содержит 'r'
SELECT name, login, email, department
FROM users
WHERE name NOT LIKE '%a'
  AND login LIKE '%r%';
