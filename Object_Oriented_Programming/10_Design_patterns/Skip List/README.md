# Задача «Skip List»

Реализовать структуру данных Skip list (список пропусков).

Подробное описание, как она работает, есть на <a href="https://habr.com/ru/articles/230413/">Хабре</a>.

Интерфейс класса подобен <a href="https://cplusplus.com/reference/map/map/">интерфейсу</a>
`std::map`, с теми же параметрами шаблона:

```cpp
template <typename Key,
    typename Value,
    typename Compare = std::less<Key>,
    typename Alloc = std::allocator<std::pair<const Key,Value> > >
class skip_list {
    // ...
public:
    typedef ... iterator;
    typedef ... const_iterator;
    typedef std::pair<const Key, Value> value_type;

    skip_list();
    explicit skip_list(const Compare &comp, const Alloc &a = Alloc());
    skip_list(const skip_list &another);

    skip_list &operator=(const skip_list &another);

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

    void swap(skip_list &another);
    void clear();

    iterator find(const Key &key);
    const_iterator find(const Key &key) const;
};

template <typename K, typename V, typename C, typename A>
inline bool operator==(const skip_list<K,V,C,A> &x, const skip_list<K,V,C,A> &y) {
    // ....
}

template <typename K, typename V, typename C, typename A>
inline bool operator!=(const skip_list<K,V,C,A> &x, const skip_list<K,V,C,A> &y) {
    // ....
}
```

### Усложнение

Реализовать ещё несколько методов из `map`:

```cpp
// ....
typedef .... reverse_iterator;
typedef .... const_reverse_iterator;

reverse_iterator rbegin();
reverse_iterator rend();
const_reverse_iterator rbegin() const;
const_reverse_iterator rend() const;

size_t count(const Key &key) const;
```

### Тестирование

Все операции должны быть проверены.
