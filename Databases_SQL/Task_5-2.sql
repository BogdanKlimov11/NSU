-- Удаление таблиц, если они существуют
DROP TABLE IF EXISTS table_one, table_two CASCADE;

-- Создание таблицы table_one
CREATE TABLE table_one (
    value_a  BIGINT,
    value_b  BIGINT
);

-- Создание таблицы table_two
CREATE TABLE table_two (
    value_a  BIGINT,
    value_b  BIGINT
);

-- Удаление функции foo, если она существует
DROP FUNCTION IF EXISTS execute_foo() CASCADE;

-- Создание функции для вставки данных с использованием точки сохранения
CREATE OR REPLACE FUNCTION execute_foo()
RETURNS BIGINT AS $$
BEGIN
    INSERT INTO table_one (value_a, value_b) VALUES (1, NULL);
    SAVEPOINT my_savepoint;
    INSERT INTO table_one (value_a, value_b) VALUES (2, NULL);
    ROLLBACK TO SAVEPOINT my_savepoint;
    INSERT INTO table_one (value_a, value_b) VALUES (3, NULL);
    RETURN 1;
EXCEPTION
    WHEN OTHERS THEN
        RETURN 0;
END;
$$ LANGUAGE plpgsql;

-- Выполнение функции execute_foo
SELECT execute_foo() AS result;

-- Вставка тестовых данных в table_one
INSERT INTO table_one (value_a, value_b)
VALUES (2, 2);

-- Выборка всех данных из table_one
SELECT * FROM table_one;

-- Удаление функций и триггеров для каскадного удаления
DROP FUNCTION IF EXISTS delete_from_table_two() CASCADE;
DROP FUNCTION IF EXISTS delete_from_table_one() CASCADE;
DROP TRIGGER IF EXISTS trigger_table_one ON table_one;
DROP TRIGGER IF EXISTS trigger_table_two ON table_two;

-- Создание функции для удаления связанных записей из table_two
CREATE OR REPLACE FUNCTION delete_from_table_two()
RETURNS TRIGGER AS $$
BEGIN
    -- Предотвращение циклической рекурсии
    IF NOT EXISTS (
        SELECT 1 
        FROM table_two 
        WHERE value_a = OLD.value_a 
        AND pg_trigger_depth() < 2
    ) THEN
        RETURN OLD;
    END IF;

    DELETE FROM table_two WHERE value_a = OLD.value_a;
    RETURN OLD;
END;
$$ LANGUAGE plpgsql;

-- Создание функции для удаления связанных записей из table_one
CREATE OR REPLACE FUNCTION delete_from_table_one()
RETURNS TRIGGER AS $$
BEGIN
    -- Предотвращение циклической рекурсии
    IF NOT EXISTS (
        SELECT 1 
        FROM table_one 
        WHERE value_a = OLD.value_a 
        AND pg_trigger_depth() < 2
    ) THEN
        RETURN OLD;
    END IF;

    DELETE FROM table_one WHERE value_a = OLD.value_a;
    RETURN OLD;
END;
$$ LANGUAGE plpgsql;

-- Создание триггера для table_one
CREATE TRIGGER trigger_table_one
    BEFORE DELETE ON table_one
    FOR EACH ROW
    EXECUTE FUNCTION delete_from_table_two();

-- Создание триггера для table_two
CREATE TRIGGER trigger_table_two
    BEFORE DELETE ON table_two
    FOR EACH ROW
    EXECUTE FUNCTION delete_from_table_one();

-- Вставка тестовых данных в table_one
INSERT INTO table_one (value_a, value_b)
VALUES 
    (1, 1),
    (2, 2);

-- Вставка тестовых данных в table_two
INSERT INTO table_two (value_a, value_b)
VALUES 
    (1, 1),
    (2, 2);

-- Удаление записи из table_one
DELETE FROM table_one WHERE value_a = 1;

-- Выборка всех данных из table_one
SELECT * FROM table_one;

-- Удаление таблицы recursion, если она существует
DROP TABLE IF EXISTS recursion CASCADE;

-- Создание таблицы для тестирования рекурсии
CREATE TABLE recursion (
    id    BIGINT GENERATED ALWAYS AS IDENTITY,
    data  VARCHAR(80) NOT NULL,
    CONSTRAINT recursion_pk PRIMARY KEY (id)
);

-- Вставка тестовых данных в recursion
INSERT INTO recursion (data)
VALUES ('20');

-- Удаление функции и триггера для обновления
DROP FUNCTION IF EXISTS update_recursion() CASCADE;
DROP TRIGGER IF EXISTS trigger_recursion ON recursion;

-- Создание функции для обновления данных с предотвращением рекурсии
CREATE OR REPLACE FUNCTION update_recursion()
RETURNS TRIGGER AS $$
BEGIN
    -- Предотвращение бесконечной рекурсии
    IF pg_trigger_depth() > 1 THEN
        RETURN NEW;
    END IF;

    -- Выполнение обновления только один раз
    UPDATE recursion 
    SET data = 'LOOOOOOOP'
    WHERE id = OLD.id;
    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

-- Создание триггера для таблицы recursion
CREATE TRIGGER trigger_recursion
    BEFORE UPDATE ON recursion
    FOR EACH ROW
    EXECUTE FUNCTION update_recursion();

-- Тестовое обновление данных в recursion
UPDATE recursion
SET data = 'start_loop'
WHERE id = 1;

-- Выборка всех данных из recursion
SELECT * FROM recursion;
