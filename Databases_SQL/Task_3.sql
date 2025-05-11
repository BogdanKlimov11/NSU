-- 3-1: Управление целостностью данных

-- Удаление таблиц, если они существуют
DROP TABLE IF EXISTS parent_table, child_table CASCADE;

-- Создание таблицы parent_table
CREATE TABLE parent_table (
    id    BIGINT GENERATED ALWAYS AS IDENTITY,
    data  VARCHAR(200) NOT NULL,
    CONSTRAINT parent_table_pk PRIMARY KEY (id)
);

-- Создание таблицы child_table
CREATE TABLE child_table (
    id        BIGINT GENERATED ALWAYS AS IDENTITY,
    data      VARCHAR(32) NOT NULL,
    parent_id BIGINT NOT NULL,
    CONSTRAINT child_table_pk PRIMARY KEY (id),
    CONSTRAINT child_table_parent_fk FOREIGN KEY (parent_id) REFERENCES parent_table (id) ON DELETE CASCADE
);

-- Вставка тестовых данных в child_table
INSERT INTO child_table (data, parent_id)
VALUES 
    ('alice', 1),
    ('bob', 2);

-- Вставка тестовых данных в parent_table
INSERT INTO parent_table (data)
VALUES 
    ('alice'),
    ('bob'),
    ('charlie'),
    ('dave');

-- Вставка дополнительных данных в child_table
INSERT INTO child_table (data, parent_id)
VALUES 
    ('alise', 1),
    ('bob', 2);

-- Попытка удаления записи из parent_table (не вызовет ошибку благодаря ON DELETE CASCADE)
DELETE FROM parent_table
WHERE data = 'alice';

-- Попытка обновления id в parent_table (вызовет ошибку без триггера)
UPDATE parent_table
SET id = 2019
WHERE id = 1;

-- Удаление триггеров и функций, если они существуют
DROP TRIGGER IF EXISTS trigger_delete_parent ON parent_table;
DROP TRIGGER IF EXISTS trigger_update_parent ON parent_table;
DROP FUNCTION IF EXISTS handle_parent_delete() CASCADE;
DROP FUNCTION IF EXISTS handle_parent_update() CASCADE;

-- Создание функции для обработки удаления из parent_table
CREATE OR REPLACE FUNCTION handle_parent_delete()
RETURNS TRIGGER AS $$
BEGIN
    -- Избыточно, так как ON DELETE CASCADE уже выполняет это
    DELETE FROM child_table
    WHERE parent_id = OLD.id;
    RETURN OLD;
END;
$$ LANGUAGE plpgsql;

-- Создание функции для обработки обновления id в parent_table
CREATE OR REPLACE FUNCTION handle_parent_update()
RETURNS TRIGGER AS $$
BEGIN
    UPDATE child_table
    SET parent_id = NEW.id
    WHERE parent_id = OLD.id;
    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

-- Создание триггера для удаления связанных записей
CREATE TRIGGER trigger_delete_parent
    BEFORE DELETE ON parent_table
    FOR EACH ROW
    EXECUTE FUNCTION handle_parent_delete();

-- Создание триггера для обновления parent_id в child_table
CREATE TRIGGER trigger_update_parent
    BEFORE UPDATE ON parent_table
    FOR EACH ROW
    WHEN (OLD.id IS DISTINCT FROM NEW.id)
    EXECUTE FUNCTION handle_parent_update();

-- Вставка тестовых данных для проверки триггеров
INSERT INTO parent_table (data)
VALUES 
    ('alice'),
    ('bob'),
    ('charlie'),
    ('dave');

INSERT INTO child_table (data, parent_id)
VALUES 
    ('alice', 5),
    ('bob', 6),
    ('some_data', 5),
    ('for_test', 6);

-- Проверка удаления записи
DELETE FROM parent_table
WHERE data = 'alice';

-- Проверка обновления id
UPDATE parent_table
SET id = 2019
WHERE id = 6;

-- 3-2: Демонстрация связей между таблицами

-- Удаление таблиц, если они существуют
DROP TABLE IF EXISTS entity_a, entity_b, entity_a_b_link CASCADE;

-- One-to-One: Создание таблиц
CREATE TABLE entity_a (
    id  BIGINT GENERATED ALWAYS AS IDENTITY,
    CONSTRAINT entity_a_pk PRIMARY KEY (id)
);

CREATE TABLE entity_b (
    id  BIGINT NOT NULL,
    CONSTRAINT entity_b_pk PRIMARY KEY (id),
    CONSTRAINT entity_b_a_fk FOREIGN KEY (id) REFERENCES entity_a (id)
);

-- One-to-Many: Создание таблиц
CREATE TABLE entity_a (
    id  BIGINT GENERATED ALWAYS AS IDENTITY,
    CONSTRAINT entity_a_pk PRIMARY KEY (id)
);

CREATE TABLE entity_b (
    id      BIGINT GENERATED ALWAYS AS IDENTITY,
    a_id    BIGINT NOT NULL,
    CONSTRAINT entity_b_pk PRIMARY KEY (id),
    CONSTRAINT entity_b_a_fk FOREIGN KEY (a_id) REFERENCES entity_a (id)
);

-- Many-to-Many: Создание таблиц
CREATE TABLE entity_a (
    id  BIGINT GENERATED ALWAYS AS IDENTITY,
    CONSTRAINT entity_a_pk PRIMARY KEY (id)
);

CREATE TABLE entity_b (
    id  BIGINT GENERATED ALWAYS AS IDENTITY,
    CONSTRAINT entity_b_pk PRIMARY KEY (id)
);

CREATE TABLE entity_a_b_link (
    a_id  BIGINT NOT NULL,
    b_id  BIGINT NOT NULL,
    CONSTRAINT entity_a_b_link_pk PRIMARY KEY (a_id, b_id),
    CONSTRAINT entity_a_b_link_a_fk FOREIGN KEY (a_id) REFERENCES entity_a (id),
    CONSTRAINT entity_a_b_link_b_fk FOREIGN KEY (b_id) REFERENCES entity_b (id)
);

-- 3-3: Создание таблицы workers

-- Удаление таблицы, если она существует
DROP TABLE IF EXISTS workers CASCADE;

-- Создание таблицы workers
CREATE TABLE workers (
    id              BIGINT GENERATED ALWAYS AS IDENTITY,
    name            VARCHAR(50) NOT NULL,
    department      VARCHAR(50) NOT NULL,
    phone           VARCHAR(20) NOT NULL,
    task            VARCHAR(100),
    books           VARCHAR(200),
    colleagues      VARCHAR(200),
    cluster_access  BOOLEAN NOT NULL,
    CONSTRAINT workers_pk PRIMARY KEY (id),
    CONSTRAINT workers_phone_unique UNIQUE (phone)
);

-- Вставка тестовых данных в workers
INSERT INTO workers (name, department, phone, task, books, colleagues, cluster_access)
VALUES 
    ('Daniil Yakovlev', 'Yandex', '79831033794', 'Decay Tree Fitter', 'Avery, Landau, CernROOT', 'Arsenty Melnikov, Pavel Lisenkov, vvorob, krokovny', TRUE),
    ('Pavel Lisenkov', 'JetBrains', '79888888888', 'Build plugin', 'Manual', 'Daniil Yakovlev, Arsenty Melnikov', TRUE),
    ('Arsenty Melnikov', 'JetBrains', '79999999999', 'Optimization TensorFlow', 'TF, keras', 'Daniil Yakovlev, Pavel Lisenkov', TRUE);
