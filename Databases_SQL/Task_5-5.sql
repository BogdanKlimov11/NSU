-- Удаление таблиц, если они существуют
DROP TABLE IF EXISTS _Users, _Tasks, _Projects CASCADE;

-- Создание таблицы пользователей
CREATE TABLE _Users
(
    name       VARCHAR(32) NOT NULL,
    login      VARCHAR(32),
    email      VARCHAR(32),
    department VARCHAR(16) CHECK (department in ('Production', 'Support', 'Accounting', 'Administration')),
    PRIMARY KEY (login)
);

-- Создание таблицы проектов
CREATE TABLE _Projects
(
    name        VARCHAR(32),
    description TEXT,
    date_on     DATE NOT NULL,
    date_off    DATE,
    PRIMARY KEY (name)
);

-- Создание таблицы задач
CREATE TABLE _Tasks
(
    project     VARCHAR(32),
    header      VARCHAR(32) NOT NULL,
    priority    INTEGER NOT NULL,
    description TEXT,
    status      VARCHAR(16) CHECK (status in ('new', 'reopened', 'closed', 'in process')) NOT NULL,
    estimate    INTEGER,
    spent       INTEGER,
    creator     VARCHAR(32),
    responsible VARCHAR(32),
    sur_key     SERIAL, -- Автоинкрементный ключ
    date_on     DATE,
    FOREIGN KEY (project) REFERENCES _Projects (name),
    FOREIGN KEY (creator) REFERENCES _Users (login),
    FOREIGN KEY (responsible) REFERENCES _Users (login),
    PRIMARY KEY (sur_key)
);

-- Вставка данных в таблицу пользователей
INSERT INTO _Users(name, login, email, department)
VALUES ('Касаткин Артем', 'a.kasatkin', 'a.kasatkin@ya.ru', 'Administration'),
       ('Петрова София', 's.petrova', 's.petrova@ya.ru', 'Accounting'),
       ('Дроздов Федр', 'f.drozdov', 'f.drozdov@ya.ru', 'Production'),
       ('Иванова Василина', 'v.ivanova', 'v.ivanova@ya.ru', 'Accounting'),
       ('Беркут Алексей', 'a.berkut', 'a.berkut@ya.ru', 'Support'),
       ('Белова Вера', 'v.belova', 'v.belova@ya.ru', 'Support'),
       ('Макенрой Алексей', 'a.makenroy', 'a.makenroy@ya.ru', 'Administration');

-- Вставка данных в таблицу проектов
INSERT INTO _Projects(name, date_on, date_off)
VALUES ('РТК', '2016/01/31', NULL),
       ('СС.Контент', '2015/02/23', '2016/12/31'),
       ('Демо-Сибирь', '2015/05/11', '2015/01/31'),
       ('МВД-Онлайн', '2015/05/22', '2016/01/31'),
       ('Поддержка', '2016/06/07', NULL);

-- Вставка данных в таблицу задач
INSERT INTO _Tasks(project, header, priority, status, creator, responsible, date_on, estimate, spent)
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

-- Выборка всех задач
SELECT * FROM _Tasks;

-- Удаление таблицы кэша задач, если существует
DROP TABLE IF EXISTS _Tasks_cache CASCADE;

-- Создание таблицы кэша задач
CREATE TABLE _Tasks_cache
(
    sur_key     SERIAL,
    t_id        INT,
    time_change TIMESTAMP NOT NULL,
    exists      BOOLEAN,
    project     VARCHAR(64) NOT NULL,
    header      VARCHAR(64),
    priority    INTEGER,
    description TEXT,
    status      VARCHAR(16) CHECK (status in ('new', 'reopened', 'closed', 'in process')),
    estimate    INTEGER,
    spent       INTEGER,
    creator     VARCHAR(32),
    responsible VARCHAR(32),
    date_on     DATE,
    FOREIGN KEY (project) REFERENCES _Projects (name),
    FOREIGN KEY (creator) REFERENCES _Users (login),
    FOREIGN KEY (responsible) REFERENCES _Users (login),
    PRIMARY KEY (sur_key)
);

-- Удаление триггера и функции модификации, если существуют
DROP TRIGGER IF EXISTS modification_trigger ON _Tasks;
DROP FUNCTION IF EXISTS modification() CASCADE;

-- Создание функции для логирования изменений в таблице задач
CREATE OR REPLACE FUNCTION modification() RETURNS TRIGGER AS
$$
BEGIN
    IF lower(tg_op) = 'insert' THEN
        INSERT INTO _Tasks_cache (t_id, time_change, exists, project, header, priority, description, status,
                                  estimate, spent, creator, responsible, date_on)
        VALUES (new.sur_key, now(), TRUE, new.project, new.header, new.priority, new.description, new.status,
                new.estimate, new.spent, new.creator, new.responsible, new.date_on);
    END IF;
    IF lower(tg_op) = 'update' THEN
        INSERT INTO _Tasks_cache (t_id, time_change, exists, project, header, priority, description, status,
                                  estimate, spent, creator, responsible, date_on)
        VALUES (new.sur_key, now(), TRUE, new.project, new.header, new.priority, new.description, new.status,
                new.estimate, new.spent, new.creator, new.responsible, new.date_on);
    END IF;
    IF lower(tg_op) = 'delete' THEN
        INSERT INTO _Tasks_cache (t_id, time_change, exists, project, header, priority, description, status,
                                  estimate, spent, creator, responsible, date_on)
        VALUES (old.sur_key, now(), FALSE, old.project, old.header, old.priority, old.description, old.status,
                old.estimate, old.spent, old.creator, old.responsible, old.date_on);
    END IF;
    RETURN new;
END;
$$ LANGUAGE plpgsql;

-- Создание триггера для вызова функции модификации
CREATE TRIGGER modification_trigger
    AFTER INSERT OR UPDATE OR DELETE
    ON _Tasks
    FOR EACH ROW
EXECUTE PROCEDURE modification();

-- Вставка тестовых задач
INSERT INTO _Tasks(project, header, priority, status, creator, responsible, date_on, estimate, spent)
VALUES ('Поддержка', 'Task122', 12, 'new', 'a.berkut', 's.petrova', NULL, 10, 15),
       ('Демо-Сибирь', 'Task22', 228, 'new', 'v.belova', 's.petrova', NULL, 52, 22);

-- Удаление задачи с приоритетом 228
DELETE FROM _Tasks WHERE priority = 228;

-- Выборка всех задач
SELECT * FROM _Tasks;

-- Выборка всех записей из кэша задач
SELECT * FROM _Tasks_cache;

-- Удаление функции истории задач, если существует
DROP FUNCTION IF EXISTS task_history(p VARCHAR, h VARCHAR);

-- Создание функции для получения истории изменений задачи
CREATE OR REPLACE FUNCTION task_history(p VARCHAR(256), h VARCHAR(128)) RETURNS SETOF _Tasks_cache AS
$$
BEGIN
    RETURN QUERY
        SELECT * FROM _Tasks_cache WHERE project = p AND h = header;
END;
$$ LANGUAGE plpgsql;

-- Обновление приоритета задачи
UPDATE _Tasks SET priority=17 WHERE header = 'Task122';

-- Получение истории изменений для задачи
SELECT * FROM task_history('Поддержка', 'Task122');

-- Удаление функции отката изменений, если существует
DROP FUNCTION IF EXISTS revert_change(rid INT);

-- Создание функции для отката изменений задачи
CREATE OR REPLACE FUNCTION revert_change(rid INT) RETURNS VARCHAR AS
$$
DECLARE
    new _Tasks_cache%ROWTYPE;
    old _Tasks_cache%ROWTYPE;
BEGIN
    SELECT * FROM _Tasks_cache
    WHERE sur_key = (SELECT MAX(sur_key) FROM _Tasks_cache AS int WHERE t_id = rid) INTO new;
    SELECT *
    FROM _Tasks_cache
    WHERE project = new.project
      AND header = new.header
      AND time_change <= new.time_change
      AND sur_key < new.sur_key
    ORDER BY time_change DESC
    LIMIT 1 INTO OLD;

    IF new IS NULL THEN
        RETURN 'Table in the same state';
    END IF;
    IF old IS NULL THEN
        DELETE FROM _Tasks WHERE header = new.header AND project = new.project;
        RETURN 'Restore failed';
    END IF;
    IF old.exists AND not new.exists THEN
        INSERT INTO _Tasks(sur_key, project, header, priority, status, creator, responsible, date_on, estimate, spent)
        VALUES (old.t_id, old.project, old.header, old.priority, old.status, old.creator, old.responsible,
                old.date_on, old.estimate, old.spent);
    ELSE
        IF NOT old.exists AND new.exists THEN
            DELETE FROM _Tasks WHERE header = old.header AND project = old.project;
        ELSE
            IF old.exists AND new.exists THEN
                UPDATE _Tasks
                SET
                    sur_key = old.t_id,
                    priority    = old.priority,
                    description = old.description,
                    status      = old.status,
                    estimate    = old.estimate,
                    spent       = old.spent,
                    creator     = old.creator,
                    responsible = old.responsible,
                    date_on     = old.date_on
                WHERE sur_key = new.t_id;
            ELSE
                RETURN 'Restore failed';
            END IF;
        END IF;
    END IF;
    RETURN 'Recovery was successful';
END;
$$ LANGUAGE plpgsql;

-- Выборка записей из кэша задач
SELECT * FROM _Tasks_cache;

-- Обновление приоритета задачи
UPDATE _Tasks SET priority=99 WHERE header = 'Task122';

-- Выборка записей из кэша задач
SELECT * FROM _Tasks_cache;

-- Выполнение отката изменений для задачи
SELECT * FROM revert_change(16);

-- Выборка записей из кэша задач
SELECT * FROM _Tasks_cache;

-- Выборка всех задач
SELECT * FROM _Tasks;

-- Удаление задачи
DELETE FROM _Tasks WHERE header = 'Task122';

-- Выборка всех задач
SELECT * FROM _Tasks;

-- Выборка записей из кэша задач
SELECT * FROM _Tasks_cache;

-- Выполнение отката изменений для задачи
SELECT * FROM revert_change(16);

-- Выборка всех задач
SELECT * FROM _Tasks;
