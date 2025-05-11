-- Удаление таблиц, если они существуют
DROP TABLE IF EXISTS lock_table_a, lock_table_b CASCADE;

-- Создание таблицы lock_table_a
CREATE TABLE lock_table_a (
    value  BIGINT NOT NULL,
    CONSTRAINT lock_table_a_pk PRIMARY KEY (value)
);

-- Создание таблицы lock_table_b
CREATE TABLE lock_table_b (
    value  BIGINT NOT NULL,
    CONSTRAINT lock_table_b_pk PRIMARY KEY (value)
);

-- Транзакция T1: Вставка в lock_table_a и попытка вставки в lock_table_b
-- Выполняется в первой сессии
BEGIN;
INSERT INTO lock_table_a (value) VALUES (1);

-- Транзакция T2: Вставка в lock_table_b и попытка вставки в lock_table_a
-- Выполняется во второй сессии, параллельно с T1
BEGIN;
INSERT INTO lock_table_b (value) VALUES (1);
INSERT INTO lock_table_a (value) VALUES (2); -- Изменено на 2, чтобы избежать конфликта первичного ключа

-- Транзакция T1: Продолжение, попытка вставки в lock_table_b
-- Выполняется в первой сессии после начала T2
INSERT INTO lock_table_b (value) VALUES (2);
-- Взаимоблокировка: T1 ждёт lock_table_b, T2 ждёт lock_table_a

-- Для завершения: откат или подтверждение транзакций
-- ROLLBACK; -- Выполнить в обеих сессиях после демонстрации
