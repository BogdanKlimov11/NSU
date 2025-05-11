-- Удаление таблиц, если они существуют
DROP TABLE IF EXISTS users, tasks, projects CASCADE;

-- Создание таблицы пользователей
CREATE TABLE users (
    name        VARCHAR(32) NOT NULL,
    login       VARCHAR(32) NOT NULL,
    email       VARCHAR(32),
    department  VARCHAR(16) NOT NULL CHECK (department IN ('Production', 'Support', 'Accounting', 'Administration')),
    CONSTRAINT users_pk PRIMARY KEY (login)
);

-- Создание таблицы проектов
CREATE TABLE projects (
    name        VARCHAR(32) NOT NULL,
    description TEXT,
    date_on     DATE NOT NULL,
    date_off    DATE,
    CONSTRAINT projects_pk PRIMARY KEY (name)
);

-- Создание таблицы задач
CREATE TABLE tasks (
    project      VARCHAR(32) NOT NULL,
    header       VARCHAR(32) NOT NULL,
    priority     INTEGER NOT NULL,
    description  TEXT,
    status       VARCHAR(16) NOT NULL CHECK (status IN ('new', 'reopened', 'closed', 'in_process')),
    estimate     INTEGER,
    spent        INTEGER,
    creator      VARCHAR(32),
    responsible  VARCHAR(32),
    id           BIGINT GENERATED ALWAYS AS IDENTITY,
    date_on      DATE,
    CONSTRAINT tasks_pk PRIMARY KEY (id),
    CONSTRAINT tasks_project_fk FOREIGN KEY (project) REFERENCES projects (name),
    CONSTRAINT tasks_creator_fk FOREIGN KEY (creator) REFERENCES users (login),
    CONSTRAINT tasks_responsible_fk FOREIGN KEY (responsible) REFERENCES users (login)
);

-- Вставка данных в таблицу пользователей
INSERT INTO users (name, login, email, department)
VALUES 
    ('Касаткин Артем', 'a.kasatkin', 'a.kasatkin@ya.ru', 'Administration'),
    ('Петрова София', 's.petrova', 's.petrova@ya.ru', 'Accounting'),
    ('Дроздов Федр', 'f.drozdov', 'f.drozdov@ya.ru', 'Production'),
    ('Иванова Василина', 'v.ivanova', 'v.ivanova@ya.ru', 'Accounting'),
    ('Беркут Алексей', 'a.berkut', 'a.berkut@ya.ru', 'Support'),
    ('Белова Вера', 'v.belova', 'v.belova@ya.ru', 'Support'),
    ('Макенрой Алексей', 'a.makenroy', 'a.makenroy@ya.ru', 'Administration');

-- Вставка данных в таблицу проектов
INSERT INTO projects (name, date_on, date_off)
VALUES 
    ('РТК', '2025-01-31', NULL),
    ('СС.Контент', '2025-02-23', '2025-12-31'),
    ('Демо-Сибирь', '2025-05-11', '2025-01-31'),
    ('МВД-Онлайн', '2025-05-22', '2025-01-31'),
    ('Поддержка', '2025-06-07', NULL);

-- Вставка данных в таблицу задач
INSERT INTO tasks (project, header, priority, status, creator, responsible, date_on, estimate, spent)
VALUES 
    ('Поддержка', 'Task1', 12, 'new', 'a.berkut', 's.petrova', NULL, 10, 15),
    ('Демо-Сибирь', 'Task2', 228, 'new', 'v.belova', 's.petrova', NULL, 52, 22),
    ('РТК', 'Task3', 1337, 'new', 'a.makenroy', 's.petrova', NULL, 12, 12),
    ('Демо-Сибирь', 'Task4', 10, 'new', 'v.ivanova', 'a.makenroy', NULL, 1, 100),
    ('МВД-Онлайн', 'Task5', 61, 'new', 'a.berkut', 'f.drozdov', NULL, 12, 22),
    ('Поддержка', 'Task6', 127, 'new', 'a.makenroy', 's.petrova', NULL, 12, 12),
    ('РТК', 'Task7', 19, 'new', 'a.makenroy', 'v.belova', NULL, 22, 35),
    ('Демо-Сибирь', 'Task8', 'in_process', 'new', 'a.makenroy', 's.petrova', NULL, 94, 12),
    ('МВД-Онлайн', 'Task9', 1, 'new', 'v.ivanova', 'f.drozdov', NULL, 88, 24),
    ('Демо-Сибирь', 'Task10', 11, 'new', 'a.makenroy', 'a.kasatkin', '2025-01-01', 99, 2),
    ('РТК', 'Task11', 22, 'new', 'v.ivanova', 'a.berkut', '2025-04-01', NULL, NULL),
    ('Демо-Сибирь', 'Task12', 3, 'new', 'a.makenroy', 'a.makenroy', '2025-08-01', 66, 32),
    ('СС.Контент', 'Task13', 1, 'new', 'a.makenroy', 'a.kasatkin', '2025-08-02', 99, 2),
    ('СС.Контент', 'Task14', 20, 'new', 'a.makenroy', 'a.kasatkin', '2025-08-03', 22, 3),
    ('СС.Контент', 'Task15', 20, 'new', 'a.makenroy', NULL, '2025-12-03', NULL, NULL);

-- Выборка всех задач
SELECT * FROM tasks;

-- Удаление таблицы кэша задач, если она существует
DROP TABLE IF EXISTS tasks_cache CASCADE;

-- Создание таблицы кэша задач
CREATE TABLE tasks_cache (
    id           BIGINT GENERATED ALWAYS AS IDENTITY,
    task_id      BIGINT,
    time_change  TIMESTAMP WITH TIME ZONE NOT NULL,
    exists       BOOLEAN,
    project      VARCHAR(64) NOT NULL,
    header       VARCHAR(64),
    priority     INTEGER,
    description  TEXT,
    status       VARCHAR(16) CHECK (status IN ('new', 'reopened', 'closed', 'in_process')),
    estimate     INTEGER,
    spent        INTEGER,
    creator      VARCHAR(32),
    responsible  VARCHAR(32),
    date_on      DATE,
    CONSTRAINT tasks_cache_pk PRIMARY KEY (id),
    CONSTRAINT tasks_cache_project_fk FOREIGN KEY (project) REFERENCES projects (name),
    CONSTRAINT tasks_cache_creator_fk FOREIGN KEY (creator) REFERENCES users (login),
    CONSTRAINT tasks_cache_responsible_fk FOREIGN KEY (responsible) REFERENCES users (login)
);

-- Удаление триггера и функции модификации, если они существуют
DROP TRIGGER IF EXISTS modification_trigger ON tasks;
DROP FUNCTION IF EXISTS modification() CASCADE;

-- Создание функции для логирования изменений в таблице задач
CREATE OR REPLACE FUNCTION log_task_modification()
RETURNS TRIGGER AS $$
BEGIN
    IF TG_OP = 'INSERT' THEN
        INSERT INTO tasks_cache (
            task_id, time_change, exists, project, header, priority, description, status,
            estimate, spent, creator, responsible, date_on
        )
        VALUES (
            NEW.id, CURRENT_TIMESTAMP, TRUE, NEW.project, NEW.header, NEW.priority, NEW.description, NEW.status,
            NEW.estimate, NEW.spent, NEW.creator, NEW.responsible, NEW.date_on
        );
    ELSIF TG_OP = 'UPDATE' THEN
        INSERT INTO tasks_cache (
            task_id, time_change, exists, project, header, priority, description, status,
            estimate, spent, creator, responsible, date_on
        )
        VALUES (
            NEW.id, CURRENT_TIMESTAMP, TRUE, NEW.project, NEW.header, NEW.priority, NEW.description, NEW.status,
            NEW.estimate, NEW.spent, NEW.creator, NEW.responsible, NEW.date_on
        );
    ELSIF TG_OP = 'DELETE' THEN
        INSERT INTO tasks_cache (
            task_id, time_change, exists, project, header, priority, description, status,
            estimate, spent, creator, responsible, date_on
        )
        VALUES (
            OLD.id, CURRENT_TIMESTAMP, FALSE, OLD.project, OLD.header, OLD.priority, OLD.description, OLD.status,
            OLD.estimate, OLD.spent, OLD.creator, OLD.responsible, OLD.date_on
        );
    END IF;
    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

-- Создание триггера для логирования изменений
CREATE TRIGGER modification_trigger
    AFTER INSERT OR UPDATE OR DELETE ON tasks
    FOR EACH ROW EXECUTE FUNCTION log_task_modification();

-- Вставка тестовых задач
INSERT INTO tasks (project, header, priority, status, creator, responsible, date_on, estimate, spent)
VALUES 
    ('Поддержка', 'Task122', 12, 'new', 'a.berkut', 's.petrova', NULL, 10, 15),
    ('Демо-Сибирь', 'Task22', 228, 'new', 'v.belova', 's.petrova', NULL, 52, 22);

-- Удаление задачи с приоритетом 228
DELETE FROM tasks WHERE priority = 228;

-- Выборка всех задач
SELECT * FROM tasks;

-- Выборка всех записей из кэша задач
SELECT * FROM tasks_cache;

-- Удаление функции истории задач, если она существует
DROP FUNCTION IF EXISTS task_history(VARCHAR, VARCHAR) CASCADE;

-- Создание функции для получения истории изменений задачи
CREATE OR REPLACE FUNCTION get_task_history(project_name VARCHAR(256), task_header VARCHAR(128))
RETURNS SETOF tasks_cache AS $$
BEGIN
    RETURN QUERY
    SELECT * FROM tasks_cache 
    WHERE project = project_name AND header = task_header;
END;
$$ LANGUAGE plpgsql;

-- Обновление приоритета задачи
UPDATE tasks 
SET priority = 17 
WHERE header = 'Task122';

-- Получение истории изменений для задачи
SELECT * FROM get_task_history('Поддержка', 'Task122');

-- Удаление функции отката изменений, если она существует
DROP FUNCTION IF EXISTS revert_task_change(INT) CASCADE;

-- Создание функции для отката изменений задачи
CREATE OR REPLACE FUNCTION revert_task_change(task_id INT)
RETURNS VARCHAR AS $$
DECLARE
    latest_record tasks_cache%ROWTYPE;
    previous_record tasks_cache%ROWTYPE;
BEGIN
    -- Получение последней записи для задачи
    SELECT * INTO latest_record
    FROM tasks_cache
    WHERE id = (
        SELECT MAX(id) 
        FROM tasks_cache 
        WHERE task_id = revert_task_change.task_id
    );

    -- Получение предыдущей записи для задачи
    SELECT * INTO previous_record
    FROM tasks_cache
    WHERE project = latest_record.project
      AND header = latest_record.header
      AND time_change <= latest_record.time_change
      AND id < latest_record.id
    ORDER BY time_change DESC
    LIMIT 1;

    IF latest_record IS NULL THEN
        RETURN 'Table in the same state';
    END IF;

    IF previous_record IS NULL THEN
        DELETE FROM tasks 
        WHERE header = latest_record.header 
          AND project = latest_record.project;
        RETURN 'Restore failed';
    END IF;

    IF previous_record.exists AND NOT latest_record.exists THEN
        INSERT INTO tasks (
            id, project, header, priority, status, creator, responsible, date_on, estimate, spent
        )
        VALUES (
            previous_record.task_id, previous_record.project, previous_record.header, 
            previous_record.priority, previous_record.status, previous_record.creator, 
            previous_record.responsible, previous_record.date_on, previous_record.estimate, 
            previous_record.spent
        );
    ELSIF NOT previous_record.exists AND latest_record.exists THEN
        DELETE FROM tasks 
        WHERE header = previous_record.header 
          AND project = previous_record.project;
    ELSIF previous_record.exists AND latest_record.exists THEN
        UPDATE tasks
        SET 
            id = previous_record.task_id,
            priority = previous_record.priority,
            description = previous_record.description,
            status = previous_record.status,
            estimate = previous_record.estimate,
            spent = previous_record.spent,
            creator = previous_record.creator,
            responsible = previous_record.responsible,
            date_on = previous_record.date_on
        WHERE id = latest_record.task_id;
    ELSE
        RETURN 'Restore failed';
    END IF;

    RETURN 'Recovery was successful';
END;
$$ LANGUAGE plpgsql;

-- Выборка записей из кэша задач
SELECT * FROM tasks_cache;

-- Обновление приоритета задачи
UPDATE tasks 
SET priority = 99 
WHERE header = 'Task122';

-- Выборка записей из кэша задач
SELECT * FROM tasks_cache;

-- Выполнение отката изменений для задачи
SELECT * FROM revert_task_change(16);

-- Выборка записей из кэша задач
SELECT * FROM tasks_cache;

-- Выборка всех задач
SELECT * FROM tasks;

-- Удаление задачи
DELETE FROM tasks 
WHERE header = 'Task122';

-- Выборка всех задач
SELECT * FROM tasks;

-- Выборка записей из кэша задач
SELECT * FROM tasks_cache;

-- Выполнение отката изменений для задачи
SELECT * FROM revert_task_change(16);

-- Выборка всех задач
SELECT * FROM tasks;
