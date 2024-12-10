-- 5.4: INITIALIZE

DROP TABLE IF EXISTS _files, _nodes CASCADE;

CREATE TABLE _nodes
(
    id   SERIAL NOT NULL PRIMARY KEY,
    path VARCHAR(100)
);

CREATE TABLE _files
(
    id       SERIAL  NOT NULL,
    name     VARCHAR NOT NULL,
    pid      INT     NOT NULL,
    node_id  INT     NOT NULL,
    size     INT,
    created  DATE,
    written  DATE    NOT NULL,
    modified DATE,
    PRIMARY KEY (id, name, node_id),
    FOREIGN KEY (node_id) REFERENCES _nodes (id)
);

INSERT INTO _nodes (path)
VALUES ('first comp'),
       ('second comp'),
       ('server');

INSERT INTO _files (name, pid, node_id, size, created, written, modified)
VALUES ('new folder', 0, 1, 1000, '2018-01-01', '2018-05-23', '2018-01-01'),
       ('mycomp', 0, 1, 1000, '2018-01-01', '2018-05-23', '2018-01-01'),
       ('sql', 1, 2, 1000, '2018-01-01', '2018-05-23', '2018-01-01'),
       ('sql1', 1, 1, 1000, '2018-01-01', '2018-05-23', '2018-01-01'),
       ('sql2', 1, 2, 1000, '2018-01-01', '2018-05-23', '2018-01-01'),
       ('sql3', 1, 3, 1000, '2018-01-01', '2018-05-23', '2018-01-01'),
       ('sql', 2, 2, 1000, '2018-01-01', '2018-05-23', '2018-01-01'),
       ('old', 2, 2, 1000, '2018-01-01', '2018-05-23', '2018-01-01');

--5.4: functions

CREATE OR REPLACE FUNCTION get_id(path VARCHAR, dir INT)
    RETURNS INT AS
$$
DECLARE
    pos INT;
    res INT;
BEGIN
    IF path = '' THEN RETURN dir; END IF;
    SELECT position('/' in path) INTO pos;
    IF pos = 0 THEN
        SELECT id FROM _files WHERE _files.pid = dir AND _files.name = path LIMIT 1 INTO res;
        RETURN coalesce(res, -1);
    ELSE
        SELECT id
        FROM _files
        WHERE _files.pid = dir
          AND _files.name = (SELECT substring(path, 1, pos - 1))
        LIMIT 1 INTO res;
        IF res IS NULL THEN
            RETURN -1;
        ELSE
            RETURN get_id((select substring(path, pos + 1)), res);
        END IF;
    END IF;
END;
$$ LANGUAGE plpgsql;

SELECT *
FROM get_id('sql', 1);
--******************************************************

CREATE OR REPLACE FUNCTION get_full_path(fid INT)
    RETURNS VARCHAR AS
$$
DECLARE
    r _files%ROWTYPE;
BEGIN
    IF NOT EXISTS(SELECT * FROM _files WHERE id = fid) THEN
        RETURN 'The file does not exist';
    ELSE
        SELECT * FROM _files WHERE id = fid INTO r;
        IF r.pid = 0 THEN
            RETURN r.name;
        ELSE
            RETURN concat(get_full_path(r.pid), '/', r.name);
        END IF;
    END IF;
END;
$$ LANGUAGE plpgsql;

SELECT *
FROM get_full_path(get_id('sql', 1));
--******************************************************

CREATE OR REPLACE FUNCTION get_depth(s VARCHAR)
    RETURNS INT AS
$$
BEGIN
    RETURN LENGTH(regexp_replace(s, '[^/]', '', 'g'));
end;
$$ LANGUAGE plpgsql;

SELECT *
FROM get_depth('a/a/a/a/a');
--******************************************************

CREATE OR REPLACE FUNCTION touch(fname VARCHAR, dirname VARCHAR, fnode INT, fsize INT, fcreated DATE, fchanged DATE)
    RETURNS VARCHAR AS
$$
DECLARE
    dir INT;
BEGIN
    IF fname LIKE '%/%' THEN RETURN 'Filename can not contain /'; END IF;
    SELECT get_id(dirname, 0) INTO dir;
    IF dir != 0 AND NOT EXISTS(SELECT * FROM _files WHERE _files.id = dir) THEN
        RETURN 'Parent directory does not exist';
    ELSE
        IF exists(SELECT * FROM _files WHERE _files.name = fname AND _files.pid = dir) THEN
            RETURN 'The file already exists';
        ELSE
            INSERT INTO _files (name, pid, node_id, size, created, written, modified)
            VALUES (fname, dir, fnode, fsize, fcreated, now(), fchanged);
            RETURN 'OK';
        END IF;
    END IF;
END;
$$ LANGUAGE plpgsql;

SELECT *
FROM touch('hello', 'new folder', 1, 1024, now()::TIMESTAMP::date, now()::TIMESTAMP::date);

SELECT *
FROM _files;
--******************************************************

CREATE OR REPLACE FUNCTION remove(fname VARCHAR)
    RETURNS VARCHAR AS
$$
DECLARE
    fid INT;
BEGIN
    SELECT get_id(fname, 0) INTO fid;
    IF fid = -1 THEN
        RETURN 'The file does not exist';
    ELSE
        IF ((SELECT COUNT(*) FROM _files WHERE pid = get_id(fname, 0)) <> 0) THEN
            RETURN 'This file have depends';
        ELSE
            DELETE FROM _files CASCADE WHERE id = fid;
            RETURN 'OK';
        END IF;
    END IF;
END;
$$ LANGUAGE plpgsql;

SELECT *
FROM _files;
SELECT *
FROM remove('new folder/sql2');
SELECT *
FROM _files;
--************************************************

CREATE OR REPLACE FUNCTION ls(dirname VARCHAR)
    RETURNS SETOF _files AS
$$
DECLARE
    r   _files%ROWTYPE;
    dir INT;
BEGIN
    SELECT get_id(dirname, 0) INTO dir;
    IF dir != 0 AND NOT EXISTS(SELECT * FROM _files WHERE _files.id = dir) THEN
        RETURN QUERY (SELECT *);
    ELSE
        FOR r IN
            SELECT * FROM _files WHERE pid = dir
            LOOP
                RETURN NEXT r;
            END LOOP;
        RETURN;
    END IF;
END;
$$ LANGUAGE plpgsql;

SELECT *
FROM ls('mycomp');
--************************************************************

CREATE OR REPLACE FUNCTION rename(fname VARCHAR, new_name VARCHAR)
    RETURNS VARCHAR AS
$$
DECLARE
    fid INT;
BEGIN
    IF new_name LIKE '%/%' THEN RETURN 'Filename can not contain /'; END IF;
    SELECT get_id(fname, 0) INTO fid;
    IF fid = -1 THEN
        RETURN 'The file does not exist';
    ELSE
        IF EXISTS(SELECT *
                  FROM _files
                  WHERE _files.name = fname
                    AND _files.pid = (SELECT pid FROM _files WHERE id = fid)) THEN
            RETURN 'The file already exists';
        ELSE
            UPDATE _files SET name = new_name, modified = now() WHERE id = fid;
            RETURN 'OK';
        END IF;
    END IF;
END;
$$ LANGUAGE plpgsql;

select *
from _files;
select *
from rename('mycomp/sql', 'nadoelo');
select *
from _files;

--************************************************************

CREATE OR REPLACE FUNCTION move(fname VARCHAR, dirname VARCHAR)
    RETURNS VARCHAR AS
$$
DECLARE
    fid INT;
    dir INT;
    fn  VARCHAR;
BEGIN
    SELECT get_id(dirname, 0) INTO dir;
    IF dir = -1 THEN
        RETURN 'New parent directory does not exist';
    ELSE
        SELECT get_id(fname, 0) INTO fid;
        SELECT right(fname, position('\' IN reverse(fname)) - 1) INTO fn;
        IF fid = -1 THEN
            RETURN 'The file does not exist';
        ELSE
            IF EXISTS(SELECT * FROM _files WHERE _files.name = fn AND _files.pid = dir) THEN
                RETURN 'The files with this name already exists in this folder';
            ELSE
                UPDATE _files SET pid = dir, modified = now() WHERE fid = id;
                RETURN 'OK';
            END IF;
        END IF;
    END IF;
END;
$$ LANGUAGE plpgsql;

SELECT *
FROM _files;
SELECT *
FROM move('mycomp/nadoelo', '');
SELECT *
FROM _files;
SELECT *
FROM ls('');

--****************--*****************--*****************--*****************

CREATE OR REPLACE FUNCTION find(mask VARCHAR, depth INT)
    RETURNS SETOF VARCHAR AS
$$
BEGIN
    RETURN QUERY
        SELECT path
        FROM (SELECT get_full_path(id) AS path FROM _files) AS p
        WHERE p.path LIKE mask
          AND (SELECT get_depth(p.path)) < depth + 1;
END;
$$ LANGUAGE plpgsql;

SELECT * from _files;

SELECT *
FROM find('%sql%', 1);
