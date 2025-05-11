from db_controller import DBController, Role


class User:
    def __init__(self, role, login, password, first_name=None, last_name=None, phone=None, **kwargs):
        self.role = role
        self.login = login
        self.password = password
        self.first_name = first_name
        self.last_name = last_name
        self.phone = phone


if __name__ == "__main__":
    # Создание пользователя-курьера
    ddyak = User(
        role=Role.courier,
        login="ddyak",
        password="hello",
        first_name="Che",
        last_name="Lal",
        phone="8"
    )
    
    # Инициализация контроллера базы данных
    db = DBController()
    
    # Регистрация пользователя
    db.register_user(ddyak)
    
    # Аутентификация пользователя
    db.authenticate_user(ddyak)
    
    # Добавление адреса
    db.add_address(street="Pirogova", number=14)
    
    # Получение всех клиентов
    db.get_all_users(Role.client)
