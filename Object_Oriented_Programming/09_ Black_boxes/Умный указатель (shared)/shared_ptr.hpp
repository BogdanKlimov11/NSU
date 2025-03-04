#pragma once

#include <functional>

template<class T, class Deleter = std::default_delete<T>>
class SharedPTR final {
    using t_SharedPTR = SharedPTR<T, Deleter>;
    using val_type = std::conditional_t<std::is_array_v<T>, typename std::remove_extent_t<T>, T>;
    using dlt_type = Deleter;

    long* m_counter = nullptr;
    val_type* m_ptr = nullptr;
    dlt_type deleter = Deleter();

    bool _assign_(const t_SharedPTR& another) {
        if (this != &another) {
            release();
            m_counter = another.m_counter;
            m_ptr = another.m_ptr;
            return true;
        }
        return false;
    }

    void incrementCount() {
        if (m_ptr) {
            (*m_counter)++;
        }
    }

public:
    SharedPTR() = default;
    explicit SharedPTR(val_type* pObj) : m_ptr(pObj) {
        if (!m_ptr) {
            return;
        }
        m_counter = new long(0);
        incrementCount();
    }
    SharedPTR(t_SharedPTR&& uniquePTR) noexcept : m_ptr(uniquePTR.m_ptr), m_counter(uniquePTR.m_counter), deleter(uniquePTR.deleter) {
        uniquePTR.m_ptr = nullptr;
        uniquePTR.m_counter = nullptr;
    }
    SharedPTR(const t_SharedPTR& sharedPtr) : m_ptr(sharedPtr.m_ptr), m_counter(sharedPtr.m_counter) {
        incrementCount();
    }
    ~SharedPTR() { release(); }

    t_SharedPTR& operator=(t_SharedPTR&& sharedPtr) noexcept {
        if (_assign_(sharedPtr)) {
            sharedPtr.m_ptr = nullptr;
            sharedPtr.m_counter = nullptr;
        }
        return *this;
    }
    t_SharedPTR& operator=(val_type* pObj) {
        *this = t_SharedPTR(pObj);
        return *this;
    }
    t_SharedPTR& operator=(const t_SharedPTR& sharedPTR) {
        if (_assign_(sharedPTR)) {
            incrementCount();
        }
        return *this;
    }

    val_type& operator*() const { return *get(); }
    val_type* operator->() const { return get(); }
    val_type* get() const { return m_ptr; }
    Deleter& get_deleter() { return deleter; }
    long use_count() const { return m_counter ? *m_counter : 0; }
    bool unique() const { return use_count() == 1; }
    operator bool() const { return m_ptr != nullptr; }

    void release() {
        if (m_counter) {
            if (*m_counter == 1) {
                deleter(m_ptr);
                delete m_counter;
                m_counter = nullptr;
                m_ptr = nullptr;
                return;
            }
            (*m_counter)--;
        }
    }

    void reset(val_type* pObj = nullptr) { *this = t_SharedPTR(pObj); }
    void swap(t_SharedPTR& sharedPTR) { std::swap(m_ptr, sharedPTR.m_ptr); std::swap(m_counter, sharedPTR.m_counter); }
};

template<class T, class ... Args>
SharedPTR<T> makeShared(Args&&... args) {
    return SharedPTR<T>(new T(std::forward<Args>(args)...));
}
