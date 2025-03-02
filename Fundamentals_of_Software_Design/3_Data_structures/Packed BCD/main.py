class Bcd:
    INVALID_BCD = 0xFFFFFFFF

    @staticmethod
    def _is_valid_bcd(n: int) -> bool:
        for i in range(7):
            if ((n >> (i * 4)) & 0xF) > 9:
                return False
        sign = (n >> 28) & 0xF
        return sign in (0xC, 0xD)

    @staticmethod
    def bcd_add(n1: int, n2: int) -> int:
        if not (Bcd._is_valid_bcd(n1) and Bcd._is_valid_bcd(n2)):
            return Bcd.INVALID_BCD
        return Bcd.bcd_from_int(Bcd.bcd_to_int(n1) + Bcd.bcd_to_int(n2))

    @staticmethod
    def bcd_sub(n1: int, n2: int) -> int:
        if not (Bcd._is_valid_bcd(n1) and Bcd._is_valid_bcd(n2)):
            return Bcd.INVALID_BCD
        return Bcd.bcd_from_int(Bcd.bcd_to_int(n1) - Bcd.bcd_to_int(n2))

    @staticmethod
    def bcd_mul(n1: int, n2: int) -> int:
        if not (Bcd._is_valid_bcd(n1) and Bcd._is_valid_bcd(n2)):
            return Bcd.INVALID_BCD
        return Bcd.bcd_from_int(Bcd.bcd_to_int(n1) * Bcd.bcd_to_int(n2))

    @staticmethod
    def bcd_div(n1: int, n2: int) -> tuple[int, int]:
        if not (Bcd._is_valid_bcd(n1) and Bcd._is_valid_bcd(n2)) or n2 == 0:
            return 0xC, 0xC  # Ошибка деления на 0
        q, r = divmod(Bcd.bcd_to_int(n1), Bcd.bcd_to_int(n2))
        return Bcd.bcd_from_int(q), Bcd.bcd_from_int(r)

    @staticmethod
    def bcd_compare(n1: int, n2: int) -> int:
        if not (Bcd._is_valid_bcd(n1) and Bcd._is_valid_bcd(n2)):
            return 0
        return (Bcd.bcd_to_int(n1) > Bcd.bcd_to_int(n2)) - (Bcd.bcd_to_int(n1) < Bcd.bcd_to_int(n2))

    @staticmethod
    def bcd_from_int(n: int) -> int:
        if abs(n) > 9999999:
            return Bcd.INVALID_BCD
        result, shift = 0, 0
        num = abs(n)
        while num:
            result |= (num % 10) << (shift * 4)
            num //= 10
            shift += 1
        sign = 0xC if n >= 0 else 0xD
        return result | (sign << 28)

    @staticmethod
    def bcd_to_int(n: int) -> int:
        if not Bcd._is_valid_bcd(n):
            return 0
        result, sign = 0, 1
        for i in range(7):
            result += ((n >> (i * 4)) & 0xF) * (10 ** i)
        if (n >> 28) & 0xF == 0xD:
            sign = -1
        return result * sign

    @staticmethod
    def bcd_to_str(n: int) -> str:
        if not Bcd._is_valid_bcd(n):
            return ""
        num = Bcd.bcd_to_int(n)
        return f"{num}"
