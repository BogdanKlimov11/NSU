import sqlite3

from enum import Enum


class Role(Enum):
    client = 1
    courier = 2

class DBcontroller:
    def __init__(self):
        self.conn = sqlite3.connect("mydatabase.db")
        self.cursor = self.conn.cursor()
        self.cursor.execute("""CREATE TABLE IF NOT EXISTS clients(
                                login       TEXT        PRIMARY KEY  NOT NULL ,
                                password    TEXT        NOT NULL,
                                firstname   TEXT,
                                lastname    TEXT,
                                phone       TEXT
                            )""")

        self.cursor.execute("""CREATE TABLE IF NOT EXISTS couriers(
                                login       TEXT        PRIMARY KEY  NOT NULL,
                                password    TEXT        NOT NULL,
                                firstName   TEXT        NOT NULL,
                                lastName    TEXT        NOT NULL,
                                phone       TEXT        NOT NULL
                            )""")

        self.cursor.execute("""CREATE TABLE IF NOT EXISTS addresses(
                                id          INTEGER     PRIMARY KEY AUTOINCREMENT,
                                number      INTEGER     NOT NULL,
                                street      TEXT        NOT NULL,
                                UNIQUE (number, street)
                            )""")

        self.cursor.execute("""CREATE TABLE IF NOT EXISTS orders(
                                id          INTEGER     PRIMARY KEY   AUTOINCREMENT,
                                time        TIMESTAMP   DEFAULT CURRENT_TIMESTAMP NOT NULL,
                                cart        TEXT        NOT NULL,
                                client_id   INTEGER     NOT NULL,
                                courier_id  INTEGER     ,
                                from_id     INTEGER     NOT NULL,
                                to_id       INTEGER     NOT NULL,
                                status      TEXT        DEFAULT 'free' CHECK (status in ('free', 'taken', 'closed')) NOT NULL,
                                FOREIGN KEY (client_id)     REFERENCES clients(id),
                                FOREIGN KEY (courier_id)    REFERENCES couriers(id),
                                FOREIGN KEY (from_id)   REFERENCES addresses(id),
                                FOREIGN KEY (to_id)    REFERENCES addresses(id)
                            )""")

    def register_user(self, user):
        if user.role == Role.client:
            sql = "INSERT INTO clients (LOGIN, PASSWORD, FIRSTNAME, LASTNAME, PHONE) VALUES (?, ?, ?, ?, ?)"
        else:
            sql = "INSERT INTO couriers (LOGIN, PASSWORD, FIRSTNAME, LASTNAME, PHONE) VALUES (?, ?, ?, ?, ?)"
        try:
            self.cursor.execute(sql, (user.login, user.password, user.firstName, user.secondName, user.phone))
        except Exception:
            print("add user: Некорректный ввод, нарушена целостность данных")
        self.conn.commit()

    def get_all_users(self, role):
        if role == Role.client:
            self.cursor.execute("""SELECT * FROM clients""")
        else:
            self.cursor.execute("""SELECT * FROM couriers""")
        rows = self.cursor.fetchall()
        for row in rows:
            print(row)

    def authentication(self, user):
        if user.role == Role.client:
            sql = "SELECT count(*) FROM clients WHERE login = ? AND password = ?"
        else:
            sql = "SELECT count(*) FROM couriers WHERE login = ? AND password = ?"
        try:
            self.cursor.execute(sql, (user.login, user.password))
            isUser = int(self.cursor.fetchall()[0][0])
            print(isUser)
        except Exception:
            print("authentication: Некорректный ввод, нарушена целостность данных")
        self.conn.commit()

    def add_address(self, street, number):
        sql = "INSERT INTO addresses (number, street) VALUES (?, ?)"
        try:
            self.cursor.execute(sql, (number, street))
        except Exception:
            print("add address: Некорректный ввод, нарушена целостность данных")
        self.conn.commit()
        self.cursor.execute("""SELECT * FROM addresses""")
        rows = self.cursor.fetchall()
        for row in rows:
            print(row)
        self.add_order('eda', 1, 1, 1)

    def add_order(self, cart, client_id, from_id, to_id):
        sql = "INSERT INTO orders (cart, client_id, from_id, to_id) VALUES (?, ?, ?, ?)"
        try:
            self.cursor.execute(sql, (cart, client_id, from_id, to_id))
        except Exception:
            print("add order:Некорректный ввод, нарушена целостность данных")
        self.conn.commit()
        self.cursor.execute("""SELECT * FROM orders""")
        rows = self.cursor.fetchall()
        for row in rows:
            print(row)
