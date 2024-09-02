from DB.DBcontroller import DBcontroller, Role

class User:
    def __init__(self, role, login, password, firstName=None, secondName=None, phone=None, **kwargs):
        self.role = role
        self.login = login
        self.password = password
        self.firstName = firstName
        self.secondName = secondName
        self.phone = phone


if __name__ == "__main__":
    ddyak = User(role=Role.courier, login='ddyak', password='hello', firstName='Che', secondName='Lal', phone='8')
    db = DBcontroller()
    db.register_user(ddyak)
    db.authentication(ddyak)
    db.add_address(14, 'Pirogova')
    db.get_all_users(Role.client)
