# Задача «Умный указатель (shared)»

### Задание

Самостоятельно реализовать <a href="https://ru.wikipedia.org/wiki/Умный_указатель">"умный
указатель"</a>, осуществляющий стратегию общего владения ресурсом (с подсчётом
ссылок), аналогичный `std::shared_ptr`.

### Описание

"Умный указатель" должен удовлетворять следующим требованиям:

* Инкапсулировать переданный в него указатель.

* Хранить внутри себя неотрицательное целое, счетчик внешних указателей,
  ссылающихся на объект.

* Когда "умный указатель" ссылается на объект, то он увеличивает счетчик
  объекта на единицу.

* Когда "умный указатель" перестает ссылаться на объект, то он уменьшает
  счетчик на единицу.

* Если счетчик становится равен нулю, то "умный указатель" удаляет объект.

* Должны быть определены операторы разыменования (* и ->).

* Должен быть следующие методы: очистка указателя, обмен с другим "умным
  указателем", инкапсуляция нового указателя.

Другой вариант реализации используется в <a href="https://www.boost.org/doc/libs/1_57_0/libs/smart_ptr/shared_ptr.htm">boost::shared_ptr</a>.
В этом случае счетчики ссылок хранятся в специальной структуре данных вне
объекта.

### Пример интерфейса

```cpp
template<class Type, class TDeleter>
class SharedPTR {
    typedef SharedPTR<Type, TDeleter> t_SharedPTR;
public: // Constructors and destructor.
    SharedPTR();
    SharedPTR(Type *pObj);
    SharedPTR(t_SharedPTR &&uniquePTR); // Move constructor.
    SharedPTR(const t_SharedPTR&);
    ~SharedPTR();
public: // Assignment.
    t_SharedPTR& operator=(t_SharedPTR &&sharedPTR);
    t_SharedPTR& operator=(Type *pObject);
    t_SharedPTR& operator=(const t_SharedPTR&);
public: // Observers.
    Type& operator*() const; // Dereference the stored pointer.
    Type* operator->() const; // Return the stored pointer.
    Type* get() const; // Return the stored pointer.
    TDeleter& get_deleter(); // Return a reference to the stored deleter.
    operator bool() const; // Return false if the stored pointer is null.
public: // Modifiers.
    void release(); // Release ownership of any stored pointer.
    void reset(Type *pObject = nullptr); // Replace the stored pointer.
    void swap(t_SharedPTR &sharedPTR); // Exchange the pointer with another object.
};
```
