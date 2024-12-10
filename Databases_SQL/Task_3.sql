--------------------- 3.1: table level

DROP TABLE IF EXISTS a, b;

CREATE TABLE a
(
  id   SERIAL PRIMARY KEY,
  data VARCHAR(200)
);

CREATE TABLE b
(
  id   SERIAL PRIMARY KEY,
  data VARCHAR(32),
  aid  INT,
  FOREIGN KEY (aid) REFERENCES a (id)
);

INSERT INTO b(data, aid)
VALUES ('alice', 1),
       ('bob', 2);

INSERT INTO a(data)
VALUES ('alice'),
       ('bob'),
       ('charlie'),
       ('dave');

INSERT INTO b(data, aid)
VALUES ('alise', 1),
       ('bob', 2);

DELETE
FROM a
WHERE data LIKE 'alice';

UPDATE a
SET id = 2019
WHERE id = 1;

--------------------- 3.1: SQL level (off tutorial PostgreSQL)

DROP TABLE IF EXISTS a, b;

CREATE TABLE a
(
  id   SERIAL PRIMARY KEY,
  data VARCHAR(32)
);

CREATE TABLE b
(
  id   SERIAL PRIMARY KEY,
  data VARCHAR(32),
  aid  INT,
  FOREIGN KEY (aid) REFERENCES a (id) ON DELETE CASCADE
);

DROP TRIGGER IF EXISTS trig_del ON a;
DROP TRIGGER IF EXISTS trig_upd ON a;

CREATE OR REPLACE FUNCTION foo() RETURNS TRIGGER AS
$$
BEGIN
  IF TG_OP = 'DELETE' THEN
    DELETE
    FROM b
    WHERE b.aid = OLD.id;
    RETURN OLD;
  END IF;
END;
$$ LANGUAGE plpgsql;

CREATE OR REPLACE FUNCTION bar() RETURNS TRIGGER AS
$$
BEGIN
  IF TG_OP = 'UPDATE' THEN
    UPDATE b
    SET aid = NEW.id
    WHERE aid = OLD.id;
    RETURN NEW;
  END IF;
END;
$$ LANGUAGE plpgsql;

CREATE TRIGGER trig_del
  BEFORE DELETE
  ON a
  FOR EACH ROW
EXECUTE PROCEDURE foo();

CREATE TRIGGER trig_upd
  AFTER UPDATE
  ON a
  FOR EACH ROW
EXECUTE PROCEDURE bar();

INSERT INTO b(data, aid)
VALUES ('alice', 1),
       ('bob', 2);

INSERT INTO a(data)
VALUES ('alice'),
       ('bob'),
       ('charlie'),
       ('dave');

INSERT INTO b(data, aid)
VALUES ('some data', 1),
       ('for test', 2);

DELETE
FROM a
WHERE data LIKE 'alice';

UPDATE a
SET id = 2019
WHERE id = 2;

----------------------------------- --3-2)

DROP TABLE IF EXISTS a, b;

-------------- one to one --------------
CREATE TABLE a
(
  id SERIAL PRIMARY KEY
);

CREATE TABLE b
(
  id INT PRIMARY KEY,
  FOREIGN KEY (id) REFERENCES a (id)
);

------------ one to many --------------
CREATE TABLE a
(
  id SERIAL PRIMARY KEY
);

CREATE TABLE b
(
  id   SERIAL PRIMARY KEY,
  a_id INT,
  FOREIGN KEY (a_id) REFERENCES a (id)
);

---------------- many to many  --------------
CREATE TABLE a
(
  id SERIAL PRIMARY KEY
);

CREATE TABLE b
(
  id SERIAL PRIMARY KEY
);

CREATE TABLE a_b_link
(
  a_id INT,
  b_id INT,
  FOREIGN KEY (a_id) REFERENCES a (id),
  FOREIGN KEY (b_id) REFERENCES b (id),
  PRIMARY KEY (a_id, b_id)
);

----------------3-3----------------
DROP TABLE IF EXISTS workers;

CREATE TABLE workers
(
  id             SERIAL PRIMARY KEY,
  name           VARCHAR(30),
  department     VARCHAR(30),
  phone          VARCHAR(11), --
  task           VARCHAR(30),
  books          VARCHAR(40),
  colleagues     VARCHAR(120),
  cluster_access BOOLEAN
);

INSERT INTO workers(name, department, phone, task, books, colleagues, cluster_access)
VALUES ('Daniil Yakovlev', 'Yandex', '79831033794', 'Decay Tree Fitter', 'Avery, Landau, CernROOT', 'Arsenty Melnikov, Pavel Lisenkov, vvorob, krokovny', TRUE),
       ('Pavel Lisenkov', 'JetBrains', '79888888888', 'Build plugin', 'Manual', 'Daniil Yakovlev, Arsenty Melnikov', TRUE),
       ('Arsenty Melnikov', 'JetBrains', '79999999999', 'Optimization TensorFlow', 'TF, keras', 'Daniil Yakovlev, Pavel Lisenkov', TRUE);


----------------------------------
