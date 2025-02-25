from collections import deque
from typing import List

class AhoCorasick:
    def __init__(self, patterns: List[str]):
        self.num_nodes = 1  # стартовая вершина
        self.edges = [{}]  # список для хранения ребер
        self.fail = [-1]  # список для хранения функций неудачи
        self.output = [{}]  # список для хранения выходных значений

        # Строим автомат
        for pattern in patterns:
            self.add_pattern(pattern)

        # Строим функции неудачи с помощью BFS
        self.build_failure()

    def add_pattern(self, pattern: str):
        current_node = 0
        for char in pattern:
            if char not in self.edges[current_node]:
                self.edges[current_node][char] = self.num_nodes
                self.num_nodes += 1
                self.edges.append({})
                self.fail.append(-1)
                self.output.append({})
            current_node = self.edges[current_node][char]
        self.output[current_node][pattern] = True

    def build_failure(self):
        queue = deque()

        # Для первого уровня в дереве указываем неудачи на корень
        for char in range(256):  # Для всех символов ASCII
            char = chr(char)
            if char in self.edges[0]:
                self.fail[self.edges[0][char]] = 0
                queue.append(self.edges[0][char])
            else:
                self.edges[0][char] = 0

        # BFS для построения всех функций неудачи
        while queue:
            current_node = queue.popleft()

            for char in self.edges[current_node]:
                fail_state = self.fail[current_node]
                while char not in self.edges[fail_state]:
                    fail_state = self.fail[fail_state]
                self.fail[self.edges[current_node][char]] = self.edges[fail_state][char]
                self.output[self.edges[current_node][char]].update(self.output[self.fail[self.edges[current_node][char]]])
                queue.append(self.edges[current_node][char])

    def search(self, text: str) -> List[int]:
        current_node = 0
        result = []

        for i in range(len(text)):
            char = text[i]
            while char not in self.edges[current_node]:
                current_node = self.fail[current_node]
            current_node = self.edges[current_node][char]
            if self.output[current_node]:
                result.append(i)  # записываем индекс конца совпадения
        return result

def substring_search(text: str, pattern: str) -> List[int]:
    ac = AhoCorasick([pattern])
    return ac.search(text)

# Тестирование
if __name__ == "__main__":
    text = "stog siena igolka v stoge"
    pattern = "stog"
    result = substring_search(text, pattern)
    print(f"Найдено на позициях: {result}")

    # Тест с несколькими паттернами
    patterns = ["stog", "siena", "igolka"]
    ac = AhoCorasick(patterns)
    result_multiple = ac.search(text)
    print(f"Найдено на позициях: {result_multiple}")
