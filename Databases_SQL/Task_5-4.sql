-- Удаление таблиц, если они существуют
DROP TABLE IF EXISTS files, nodes CASCADE;

-- Создание таблицы узлов
CREATE TABLE nodes (
    id    BIGINT GENERATED ALWAYS AS IDENTITY,
    path  VARCHAR(100) NOT NULL,
    CONSTRAINT nodes_pk PRIMARY KEY (id)
);

-- Создание таблицы файлов
CREATE TABLE files (
    id         BIGINT GENERATED ALWAYS AS IDENTITY,
    name       VARCHAR(255) NOT NULL,
    parent_id  BIGINT NOT NULL,
    node_id    BIGINT NOT NULL,
    size       BIGINT,
    created    DATE,
    written    DATE NOT NULL,
    modified   DATE,
    CONSTRAINT files_pk PRIMARY KEY (id, name, node_id),
    CONSTRAINT files_node_fk FOREIGN KEY (node_id) REFERENCES nodes (id)
);

-- Вставка данных в таблицу узлов
INSERT INTO nodes (path)
VALUES 
    ('first_comp'),
    ('second_comp'),
    ('server');

-- Вставка данных в таблицу файлов
INSERT INTO files (name, parent_id, node_id, size, created, written, modified)
VALUES 
    ('new_folder', 0, 1, 1000, '2025-01-01', '2025-05-23', '2025-01-01'),
    ('mycomp', 0, 1, 1000, '2025-01-01', '2025-05-23', '2025-01-01'),
    ('sql', 1, 2, 1000, '2025-01-01', '2025-05-23', '2025-01-01'),
    ('sql1', 1, 1, 1000, '2025-01-01', '2025-05-23', '2025-01-01'),
    ('sql2', 1, 2, 1000, '2025-01-01', '2025-05-23', '2025-01-01'),
    ('sql3', 1, 3, 1000, '2025-01-01', '2025-05-23', '2025-01-01'),
    ('sql', 2, 2, 1000, '2025-01-01', '2025-05-23', '2025-01-01'),
    ('old', 2, 2, 1000, '2025-01-01', '2025-05-23', '2025-01-01');

-- Функция для получения ID файла по пути
CREATE OR REPLACE FUNCTION get_file_id(file_path VARCHAR(255), directory_id BIGINT)
RETURNS BIGINT AS $$
DECLARE
    slash_pos  BIGINT;
    result_id  BIGINT;
BEGIN
    IF file_path = '' THEN 
        RETURN directory_id; 
    END IF;

    slash_pos := POSITION('/' IN file_path);
    IF slash_pos = 0 THEN
        SELECT id INTO result_id
        FROM files
        WHERE parent_id = directory_id AND name = file_path
        LIMIT 1;
        RETURN COALESCE(result_id, -1);
    ELSE
        SELECT id INTO result_id
        FROM files
        WHERE parent_id = directory_id
          AND name = SUBSTRING(file_path FROM 1 FOR slash_pos - 1)
        LIMIT 1;
        IF result_id IS NULL THEN
            RETURN -1;
        ELSE
            RETURN get_file_id(SUBSTRING(file_path FROM slash_pos + 1), result_id);
        END IF;
    END IF;
END;
$$ LANGUAGE plpgsql;

-- Тест функции get_file_id
SELECT get_file_id('sql', 1) AS file_id;

-- Функция для получения полного пути файла
CREATE OR REPLACE FUNCTION get_full_path(file_id BIGINT)
RETURNS VARCHAR(255) AS $$
DECLARE
    file_record files%ROWTYPE;
BEGIN
    IF NOT EXISTS (SELECT 1 FROM files WHERE id = file_id) THEN
        RETURN 'The file does not exist';
    END IF;

    SELECT * INTO file_record 
    FROM files 
    WHERE-letter id = file_id;

    IF file_record.parent_id = 0 THEN
        RETURN file_record.name;
    END IF;

    RETURN get_full_path(file_record.parent_id) || '/' || file_record.name;
END;
$$ LANGUAGE plpgsql;

-- Тест функции get_full_path
SELECT get_full_path(get_file_id('sql', 1)) AS full_path;

-- Функция для вычисления глубины пути
CREATE OR REPLACE FUNCTION get_path_depth(path VARCHAR(255))
RETURNS INTEGER AS $$
BEGIN
    RETURN LENGTH(REGEXP_REPLACE(path, '[^/]', '', 'g'));
END;
$$ LANGUAGE plpgsql;

-- Тест функции get_path_depth
SELECT get_path_depth('a/a/a/a/a') AS path_depth;

-- Функция для создания файла
CREATE OR REPLACE FUNCTION create_file(
    file_name VARCHAR(255), 
    dir_name VARCHAR(255), 
    node_id BIGINT, 
    file_size BIGINT, 
    created_date DATE, 
    modified_date DATE
)
RETURNS VARCHAR(255) AS $$
DECLARE
    dir_id BIGINT;
BEGIN
    IF file_name LIKE '%/%' THEN 
        RETURN 'Filename cannot contain /';
    END IF;

    dir_id := get_file_id(dir_name, 0);
    IF dir_id != 0 AND NOT EXISTS (SELECT 1 FROM files WHERE id = dir_id) THEN
        RETURN 'Parent directory does not exist';
    END IF;

    IF EXISTS (
        SELECT 1 
        FROM files 
        WHERE name = file_name AND parent_id = dir_id
    ) THEN
        RETURN 'The file already exists';
    END IF;

    INSERT INTO files (name, parent_id, node_id, size, created, written, modified)
    VALUES (file_name, dir_id, node_id, file_size, created_date, CURRENT_DATE, modified_date);

    RETURN 'OK';
END;
$$ LANGUAGE plpgsql;

-- Тест функции create_file и выборка файлов
SELECT create_file(
    'hello', 
    'new_folder', 
    1, 
    1024, 
    CURRENT_DATE, 
    CURRENT_DATE
) AS result;
SELECT * FROM files;

-- Функция для удаления файла
CREATE OR REPLACE FUNCTION remove_file(file_name VARCHAR(255))
RETURNS VARCHAR(255) AS $$
DECLARE
    file_id BIGINT;
BEGIN
    file_id := get_file_id(file_name, 0);
    IF file_id = -1 THEN
        RETURN 'The file does not exist';
    END IF;

    IF (SELECT COUNT(*) FROM files WHERE parent_id = file_id) > 0 THEN
        RETURN 'This file has dependencies';
    END IF;

    DELETE FROM files WHERE id = file_id;
    RETURN 'OK';
END;
$$ LANGUAGE plpgsql;

-- Тест функции remove_file и выборка файлов
SELECT * FROM files;
SELECT remove_file('new_folder/sql2') AS result;
SELECT * FROM files;

-- Функция для отображения содержимого директории
CREATE OR REPLACE FUNCTION list_directory(dir_name VARCHAR(255))
RETURNS SETOF files AS $$
DECLARE
    dir_id BIGINT;
BEGIN
    dir_id := get_file_id(dir_name, 0);
    IF dir_id != 0 AND NOT EXISTS (SELECT 1 FROM files WHERE id = dir_id) THEN
        RETURN QUERY SELECT * FROM files WHERE FALSE;
    END IF;

    RETURN QUERY 
    SELECT * FROM files WHERE parent_id = dir_id;
END;
$$ LANGUAGE plpgsql;

-- Тест функции list_directory
SELECT * FROM list_directory('mycomp');

-- Функция для переименования файла
CREATE OR REPLACE FUNCTION rename_file(file_name VARCHAR(255), new_name VARCHAR(255))
RETURNS VARCHAR(255) AS $$
DECLARE
    file_id BIGINT;
    parent_id BIGINT;
BEGIN
    IF new_name LIKE '%/%' THEN 
        RETURN 'Filename cannot contain /';
    END IF;

    file_id := get_file_id(file_name, 0);
    IF file_id = -1 THEN
        RETURN 'The file does not exist';
    END IF;

    SELECT parent_id INTO parent_id 
    FROM files 
    WHERE id = file_id;

    IF EXISTS (
        SELECT 1 
        FROM files 
        WHERE name = new_name AND parent_id = parent_id
    ) THEN
        RETURN 'The file already exists';
    END IF;

    UPDATE files 
    SET name = new_name, modified = CURRENT_DATE 
    WHERE id = file_id;

    RETURN 'OK';
END;
$$ LANGUAGE plpgsql;

-- Тест функции rename_file и выборка файлов
SELECT * FROM files;
SELECT rename_file('mycomp/sql', 'nadoelo') AS result;
SELECT * FROM files;

-- Функция для перемещения файла
CREATE OR REPLACE FUNCTION move_file(file_name VARCHAR(255), dir_name VARCHAR(255))
RETURNS VARCHAR(255) AS $$
DECLARE
    file_id BIGINT;
    dir_id BIGINT;
    base_name VARCHAR(255);
BEGIN
    dir_id := get_file_id(dir_name, 0);
    IF dir_id = -1 THEN
        RETURN 'New parent directory does not exist';
    END IF;

    file_id := get_file_id(file_name, 0);
    base_name := RIGHT(file_name, POSITION('/' IN REVERSE(file_name)) - 1);
    IF file_id = -1 THEN
        RETURN 'The file does not exist';
    END IF;

    IF EXISTS (
        SELECT 1 
        FROM files 
        WHERE name = base_name AND parent_id = dir_id
    ) THEN
        RETURN 'A file with this name already exists in the destination folder';
    END IF;

    UPDATE files 
    SET parent_id = dir_id, modified = CURRENT_DATE 
    WHERE id = file_id;

    RETURN 'OK';
END;
$$ LANGUAGE plpgsql;

-- Тест функции move_file и выборка файлов
SELECT * FROM files;
SELECT move_file('mycomp/nadoelo', '') AS result;
SELECT * FROM files;
SELECT * FROM list_directory('');

-- Функция для поиска файлов по маске и глубине
CREATE OR REPLACE FUNCTION find_files(search_mask VARCHAR(255), max_depth INTEGER)
RETURNS SETOF VARCHAR(255) AS $$
BEGIN
    RETURN QUERY
    SELECT full_path
    FROM (
        SELECT get_full_path(id) AS full_path 
        FROM files
    ) AS paths
    WHERE full_path LIKE search_mask
      AND get_path_depth(full_path) < max_depth + 1;
END;
$$ LANGUAGE plpgsql;

-- Тест функции find_files и выборка файлов
SELECT * FROM files;
SELECT * FROM find_files('%sql%', 1);
