DROP TABLE IF EXISTS table1, table2;

CREATE TABLE table1
(
    a INT,
    b INT
);

CREATE TABLE table2
(
    a INT,
    b INT
);

DROP FUNCTION IF EXISTS foo();

CREATE OR REPLACE FUNCTION foo(
) RETURNS int AS
$$
BEGIN
    INSERT INTO table1
    VALUES (1);
    RETURN 1;
    SAVEPOINT my_savepoint;
    INSERT INTO table1
    VALUES (2);
    ROLLBACK TO SAVEPOINT my_savepoint;
    INSERT INTO table1
    VALUES (3);
    COMMIT;
EXCEPTION
    WHEN OTHERS THEN RETURN 0;
end
$$
    LANGUAGE 'plpgsql';

SELECT *
FROM foo();

INSERT INTO table1
VALUES (2, 2);

SELECT *
FROM table1;

-- *****************      b (circle)            *****************

CREATE OR REPLACE FUNCTION delfromb() RETURNS TRIGGER AS
$$
BEGIN
    DELETE FROM table1 WHERE a = old.a;
    RETURN old;
END;
$$
    LANGUAGE plpgsql;

CREATE OR REPLACE FUNCTION delfroma() RETURNS TRIGGER AS
$$
BEGIN
    DELETE FROM table2 WHERE a = old.a;
    RETURN old;
END;
$$
    LANGUAGE plpgsql;

DROP TRIGGER IF EXISTS tg_a ON b;
DROP TRIGGER IF EXISTS tg_b ON a;

CREATE TRIGGER tg_a
    BEFORE DELETE
    ON table1
    FOR EACH ROW
EXECUTE PROCEDURE delfromb();

CREATE TRIGGER tg_b
    BEFORE DELETE
    ON table2
    FOR EACH ROW
EXECUTE PROCEDURE delfroma();

INSERT INTO table1
VALUES (1, 1);
INSERT INTO table1
VALUES (2, 2);
INSERT INTO table2
VALUES (1, 1);
INSERT INTO table2
VALUES (2, 2);

delete
from table1
where a = 1;

SELECT *
FROM table1;

-- *****************      c (recursion)         *****************

DROP TABLE IF EXISTS recursion;

CREATE TABLE IF NOT EXISTS recursion
(
    id   SERIAL PRIMARY KEY,
    data VARCHAR(80)
);

INSERT INTO recursion
VALUES (7, '20');

CREATE OR REPLACE FUNCTION upd() RETURNS TRIGGER AS
$$
BEGIN
    UPDATE recursion SET data = 'LOOOOOOOP' WHERE id = old.id;
    RETURN old;
END;
$$
    LANGUAGE plpgsql;

CREATE TRIGGER my_tgA
    BEFORE UPDATE
    ON recursion
    FOR EACH ROW
EXECUTE PROCEDURE upd();

UPDATE recursion
SET data = 'start loop'
WHERE id = 7;

SELECT *
FROM recursion;