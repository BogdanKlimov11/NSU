import psycopg2
from enum import Enum
from psycopg2.extras import RealDictCursor


class Role(Enum):
    client = 1
    courier = 2


class DBController:
    def __init__(self, dbname="mydatabase", user="postgres", password="password", host="localhost", port="5432"):
        try:
            self.conn = psycopg2.connect(
                dbname=dbname, user=user, password=password, host=host, port=port
            )
            self.cursor = self.conn.cursor(cursor_factory=RealDictCursor)
            self._initialize_database()
        except psycopg2.Error as e:
            print(f"Ошибка подключения к базе данных: {e}")
            raise

    def _initialize_database(self):
        # Создание таблицы clients
        self.cursor.execute("""
            CREATE TABLE IF NOT EXISTS clients (
                id          BIGINT GENERATED ALWAYS AS IDENTITY,
                login       VARCHAR(50) NOT NULL,
                password    VARCHAR(100) NOT NULL,
                first_name  VARCHAR(50),
                last_name   VARCHAR(50),
                phone       VARCHAR(20),
                CONSTRAINT clients_pk PRIMARY KEY (id),
                CONSTRAINT clients_login_unique UNIQUE (login)
            )
        """)

        # Создание таблицы couriers
        self.cursor.execute("""
            CREATE TABLE IF NOT EXISTS couriers (
                id          BIGINT GENERATED ALWAYS AS IDENTITY,
                login       VARCHAR(50) NOT NULL,
                password    VARCHAR(100) NOT NULL,
                first_name  VARCHAR(50) NOT NULL,
                last_name   VARCHAR(50) NOT NULL,
                phone       VARCHAR(20) NOT NULL,
                CONSTRAINT couriers_pk PRIMARY KEY (id),
                CONSTRAINT couriers_login_unique UNIQUE (login)
            )
        """)

        # Создание таблицы addresses
        self.cursor.execute("""
            CREATE TABLE IF NOT EXISTS addresses (
                id          BIGINT GENERATED ALWAYS AS IDENTITY,
                number      INTEGER NOT NULL,
                street      VARCHAR(100) NOT NULL,
                CONSTRAINT addresses_pk PRIMARY KEY (id),
                CONSTRAINT addresses_unique UNIQUE (number, street)
            )
        """)

        # Создание таблицы orders
        self.cursor.execute("""
            CREATE TABLE IF NOT EXISTS orders (
                id          BIGINT GENERATED ALWAYS AS IDENTITY,
                created_at  TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP NOT NULL,
                cart        TEXT NOT NULL,
                client_id   BIGINT NOT NULL,
                courier_id  BIGINT,
                from_id     BIGINT NOT NULL,
                to_id       BIGINT NOT NULL,
                status      VARCHAR(16) DEFAULT 'free' CHECK (status IN ('free', 'taken', 'closed')) NOT NULL,
                CONSTRAINT orders_pk PRIMARY KEY (id),
                CONSTRAINT orders_client_fk FOREIGN KEY (client_id) REFERENCES clients (id),
                CONSTRAINT orders_courier_fk FOREIGN KEY (courier_id) REFERENCES couriers (id),
                CONSTRAINT orders_from_fk FOREIGN KEY (from_id) REFERENCES addresses (id),
                CONSTRAINT orders_to_fk FOREIGN KEY (to_id) REFERENCES addresses (id)
            )
        """)

        # Создание индексов для оптимизации
        self.cursor.execute("""
            CREATE INDEX IF NOT EXISTS idx_orders_client_id ON orders (client_id);
            CREATE INDEX IF NOT EXISTS idx_orders_courier_id ON orders (courier_id);
            CREATE INDEX IF NOT EXISTS idx_orders_from_id ON orders (from_id);
            CREATE INDEX IF NOT EXISTS idx_orders_to_id ON orders (to_id);
        """)

        self.conn.commit()

    def register_user(self, user):
        if user.role == Role.client:
            sql = """
                INSERT INTO clients (login, password, first_name, last_name, phone)
                VALUES (%s, %s, %s, %s, %s)
            """
        else:
            sql = """
                INSERT INTO couriers (login, password, first_name, last_name, phone)
                VALUES (%s, %s, %s, %s, %s)
            """
        try:
            self.cursor.execute(sql, (user.login, user.password, user.first_name, user.last_name, user.phone))
            self.conn.commit()
        except psycopg2.IntegrityError as e:
            print(f"Ошибка регистрации пользователя: Нарушена целостность данных ({e})")
            self.conn.rollback()
        except psycopg2.Error as e:
            print(f"Ошибка регистрации пользователя: {e}")
            self.conn.rollback()

    def get_all_users(self, role):
        if role == Role.client:
            sql = "SELECT id, login, first_name, last_name, phone FROM clients"
        else:
            sql = "SELECT id, login, first_name, last_name, phone FROM couriers"
        try:
            self.cursor.execute(sql)
            return self.cursor.fetchall()
        except psycopg2.Error as e:
            print(f"Ошибка получения пользователей: {e}")
            return []

    def authenticate_user(self, user):
        if user.role == Role.client:
            sql = "SELECT COUNT(*) AS count FROM clients WHERE login = %s AND password = %s"
        else:
            sql = "SELECT COUNT(*) AS count FROM couriers WHERE login = %s AND password = %s"
        try:
            self.cursor.execute(sql, (user.login, user.password))
            result = self.cursor.fetchone()
            is_user = result["count"]
            print(f"Аутентификация: {'Успешно' if is_user else 'Неудачно'}")
            return is_user > 0
        except psycopg2.Error as e:
            print(f"Ошибка аутентификации: {e}")
            return False

    def add_address(self, street, number):
        sql = "INSERT INTO addresses (street, number) VALUES (%s, %s) RETURNING id"
        try:
            self.cursor.execute(sql, (street, number))
            address_id = self.cursor.fetchone()["id"]
            self.conn.commit()
            self.cursor.execute("SELECT id, number, street FROM addresses")
            rows = self.cursor.fetchall()
            for row in rows:
                print(f"Адрес: ID={row['id']}, Number={row['number']}, Street={row['street']}")
            # Добавление тестового заказа (для демонстрации)
            self.add_order(cart="eda", client_id=1, from_id=address_id, to_id=1)
        except psycopg2.IntegrityError as e:
            print(f"Ошибка добавления адреса: Нарушена целостность данных ({e})")
            self.conn.rollback()
        except psycopg2.Error as e:
            print(f"Ошибка добавления адреса: {e}")
            self.conn.rollback()

    def add_order(self, cart, client_id, from_id, to_id):
        sql = """
            INSERT INTO orders (cart, client_id, from_id, to_id)
            VALUES (%s, %s, %s, %s)
            RETURNING id
        """
        try:
            self.cursor.execute(sql, (cart, client_id, from_id, to_id))
            order_id = self.cursor.fetchone()["id"]
            self.conn.commit()
            self.cursor.execute("SELECT id, created_at, cart, client_id, courier_id, from_id, to_id, status FROM orders")
            rows = self.cursor.fetchall()
            for row in rows:
                print(f"Заказ: ID={row['id']}, Created={row['created_at']}, Cart={row['cart']}, "
                      f"ClientID={row['client_id']}, CourierID={row['courier_id']}, "
                      f"FromID={row['from_id']}, ToID={row['to_id']}, Status={row['status']}")
        except psycopg2.IntegrityError as e:
            print(f"Ошибка добавления заказа: Нарушена целостность данных ({e})")
            self.conn.rollback()
        except psycopg2.Error as e:
            print(f"Ошибка добавления заказа: {e}")
            self.conn.rollback()

    def __del__(self):
        self.cursor.close()
        self.conn.close()
