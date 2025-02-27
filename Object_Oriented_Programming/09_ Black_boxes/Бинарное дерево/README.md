# Задача «Бинарное дерево»

### Задание

Реализовать класс, реализующий несбалансированное <a href="https://ru.wikipedia.org/wiki/Двоичное_дерево_поиска">бинарное
дерево поиска</a>.

Интерфейс класса подобен интерфейсу см. <a href="https://cplusplus.com/reference/map/map/">`std::map`</a>
или <a href="https://en.cppreference.com/w/cpp/container/map">`std::map`</a>,
с теми же параметрами шаблона:

```cpp
template <typename Key,
          typename Value,
          typename Compare = std::less<Key>,
          typename Alloc = std::allocator<std::pair<const Key,Value> > >
class btree {
    // ...
public:
    using iterator =  ...;
    using const_iterator =  ...;
    using value_type = std::pair<const Key, Value>;

    btree();
    explicit btree(const Compare &comp, const Alloc &a = Alloc());
    btree(const btree &another);

    btree &operator=(const btree &another);

    iterator begin();
    const_iterator begin() const;
    iterator end();
    const_iterator end() const;

    bool empty() const;
    size_t size() const;

    Value &operator[](const Key &key);
    Value &at(const Key &key);
    const Value &at(const Key &key);

    std::pair<iterator, bool> insert(const value_type &);

    void erase(iterator position);
    size_type erase(const Key &key);
    void erase(iterator first, iterator last);

    void swap(btree &another);
    void clear();

    iterator find(const Key &key);
    const_iterator find(const Key &key) const;
};

template <typename K, typename V, typename C, typename A>
inline bool operator==(const btree<K,V,C,A> &x, const btree<K,V,C,A> &y) {
    // ....
}

template <typename K, typename V, typename C, typename A>
inline bool operator!=(const btree<K,V,C,A> &x, const btree<K,V,C,A> &y) {
    // ....
}
```

### Тестирование

Все операции должны быть проверены.
