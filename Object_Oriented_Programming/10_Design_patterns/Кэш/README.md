# Задача «Кэш»

### Справка

<a href="https://ru.wikipedia.org/wiki/Стратегия_%28шаблон_проектирования%29">Паттерн
стратегия</a>

### Задача

1. Реализовать класс кэш. Примерный интерфейс:

```cpp
class BaseCache {
public:
    virtual bool has(const std::string &key) = 0;
    virtual std::string read(const std::string &key) = 0;
    virtual void write(const std::string &key, const std::string &value) = 0;
    virtual void delete(const std::string &key) = 0;
};
```

2. Реализовать четыре типа поведения (стратегий)

* **MemoryCache** — данные хранятся в unordered_map (или map, если не с++11)

* **FileCache** — данные хранятся в файлах на диске. По ключу генерируется хэш
  (можно использовать ранее реализованные студентом хэш-функции), который
  является именем файла. В файлах хранятся tab separated пары ключ-значения
  (ключи нужны для резолвинга коллизий)

Пример файла:

```
vasja   +79454585688
petja   +79499585688
```

* **NullCache** — кэш ничего не хранит, а запись происходит дико быстро:)

* **PoorManMemoryCache** — кэш хранит последние N записанных ключей

3. Реализовать класс, который будет использовать одну из стратегий

```cpp
class CacheApplier {
public:
    void set_strategy(BaseCache& strategy);

    std::string read_from_cache(const std::string &key); 
    void write_from_cache(const std::string &key, const std::string &value)
    void delete_from_cache(const std::string &key); // Удалить из cache
};
```
